#include "robot_link.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <string.h>

#include "app/app_config.h"
#include "app/app_state.h"
#include "bus/robot_bus.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "protocol/robot_ack.h"
#include "protocol/robot_protocol.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"

static const char *TAG = "usb_chassis";

#define USB_LINK_TASK_STACK       4096
#define USB_LINK_TASK_PRIORITY    9
#define USB_CDC_TASK_PRIORITY     10
#define USB_CDC_TX_BUFFER         512
#define USB_CDC_RX_BUFFER         512
#define USB_CDC_TX_TIMEOUT_MS     250
#define USB_COMMAND_ACK_TIMEOUT_MS 250
#define USB_PRIORITY_LOCK_TIMEOUT_MS 250

typedef enum {
    USB_LINK_CONNECTED = 0,
    USB_LINK_DISCONNECTED,
} usb_link_event_t;

typedef struct {
    usb_link_event_t type;
    uint16_t vid;
    uint16_t pid;
} usb_link_message_t;

static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_command_mutex;
static SemaphoreHandle_t s_ack_semaphore;
static SemaphoreHandle_t s_usb_installed;
static QueueHandle_t s_usb_queue;
static QueueHandle_t s_params_rx;

typedef struct {
    uint8_t payload[RP_PARAM_FRAME_LEN];
} param_frame_t;
static _Atomic(cdc_acm_dev_hdl_t) s_cdc_handle;
static _Atomic(cdc_acm_dev_hdl_t) s_gone_handle;
static _Atomic bool s_cdc_ready;
static volatile bool s_link_started;
static volatile bool s_raw_rx_seen;
static rp_parser_t s_parser;
static portMUX_TYPE s_ack_lock = portMUX_INITIALIZER_UNLOCKED;
static rp_ack_latch_t s_ack;

static void set_ack_expected(uint8_t command)
{
    portENTER_CRITICAL(&s_ack_lock);
    s_ack = (rp_ack_latch_t){.command = command};
    portEXIT_CRITICAL(&s_ack_lock);
}

static rp_ack_latch_t ack_snapshot(void)
{
    portENTER_CRITICAL(&s_ack_lock);
    rp_ack_latch_t snapshot = s_ack;
    portEXIT_CRITICAL(&s_ack_lock);
    return snapshot;
}

static void clear_link_telemetry(void);

static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t length);

static void publish_topic(const char *name, const void *data, size_t len)
{
    int topic = rb_topic_find(name);
    if (topic >= 0) {
        (void)rb_publish(topic, data, len);
    }
}

static bool usb_rx_callback(const uint8_t *data, size_t data_len, void *arg)
{
    uint8_t cmd;
    uint8_t payload[RP_MAX_PAYLOAD];
    uint8_t length;
    uint32_t crc_errors = 0u;
    uint32_t length_errors = 0u;

    (void)arg;
    if (!s_cdc_ready) {
        return true;
    }
    if (data_len > 0u && !s_raw_rx_seen) {
        s_raw_rx_seen = true;
        ESP_LOGI(TAG, "first raw RX callback: %u byte(s)",
                 (unsigned)data_len);
        ESP_LOG_BUFFER_HEXDUMP(
            TAG, data, data_len < 32u ? data_len : 32u, ESP_LOG_INFO);
    }
    for (size_t i = 0; i < data_len; ++i) {
        rp_parse_result_t result =
            rp_parser_feed(&s_parser, data[i], &cmd, payload, &length);
        if (result == RP_PARSE_FRAME) {
            handle_frame(cmd, payload, length);
        } else if (result == RP_PARSE_CRC_ERROR) {
            ++crc_errors;
        } else if (result == RP_PARSE_LENGTH_ERROR) {
            ++length_errors;
        }
    }

    if (data_len > 0u) {
        app_state_lock();
        td_status_t *status = app_state_mutable();
        status->stm32_rx_bytes += (uint32_t)data_len;
        status->stm32_crc_errors += crc_errors;
        status->stm32_length_errors += length_errors;
        app_state_unlock();
    }
    return true;
}

static void usb_device_event_callback(
    const cdc_acm_host_dev_event_data_t *event,
    void *user_ctx)
{
    usb_link_message_t message = {0};
    (void)user_ctx;

    switch (event->type) {
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        message.type = USB_LINK_DISCONNECTED;
        /* Keep the detach handle even when the wake-up queue is full. */
        s_gone_handle = event->data.cdc_hdl;
        if (event->data.cdc_hdl == s_cdc_handle) {
            s_cdc_ready = false;
            set_ack_expected(0u);
            xSemaphoreGive(s_ack_semaphore);
            clear_link_telemetry();
        }
        (void)xQueueSend(s_usb_queue, &message, 0);
        break;
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC error: %d", event->data.error);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        ESP_LOGD(TAG, "CDC serial state: 0x%04X",
                 event->data.serial_state.val);
        break;
    default:
        break;
    }
}

static void usb_new_device_callback(usb_device_handle_t usb_dev)
{
    const usb_device_desc_t *device_desc;
    esp_err_t err =
        usb_host_get_device_descriptor(usb_dev, &device_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device descriptor failed: %s",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "USB device connected VID=0x%04X PID=0x%04X",
             device_desc->idVendor, device_desc->idProduct);
    if (device_desc->idVendor != TD_USB_LINK_VID ||
        device_desc->idProduct != TD_USB_LINK_PID) {
        ESP_LOGI(TAG, "ignored non-C30D USB device");
        return;
    }

    usb_link_message_t message = {
        .type = USB_LINK_CONNECTED,
        .vid = device_desc->idVendor,
        .pid = device_desc->idProduct,
    };
    if (s_usb_queue != NULL) {
        (void)xQueueSend(s_usb_queue, &message, 0);
    }
}

static esp_err_t open_cdc_device(uint16_t vid, uint16_t pid)
{
    cdc_acm_host_device_config_t config = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = USB_CDC_TX_BUFFER,
        .in_buffer_size = USB_CDC_RX_BUFFER,
        .event_cb = usb_device_event_callback,
        .data_cb = usb_rx_callback,
        .user_arg = NULL,
    };
    cdc_acm_dev_hdl_t handle = NULL;
    esp_err_t err = ESP_ERR_NOT_FOUND;

    for (uint8_t interface_idx = 0; interface_idx < 2; ++interface_idx) {
        err = cdc_acm_host_open(vid, pid, interface_idx, &config, &handle);
        if (err == ESP_OK) {
            break;
        }
    }
    if (err != ESP_OK || handle == NULL) {
        ESP_LOGE(TAG, "CDC open failed VID=0x%04X PID=0x%04X: %s",
                 vid, pid, esp_err_to_name(err));
        return err != ESP_OK ? err : ESP_FAIL;
    }

    cdc_acm_line_coding_t requested_coding = {
        .dwDTERate = TD_USB_LINK_BAUD,
        .bCharFormat = 0, /* 1 stop bit. */
        .bParityType = 0, /* No parity. */
        .bDataBits = 8,
    };
    esp_err_t coding_err =
        cdc_acm_host_line_coding_set(handle, &requested_coding);
    if (coding_err != ESP_OK) {
        ESP_LOGE(TAG, "CDC line coding set failed: %s",
                 esp_err_to_name(coding_err));
    }

    cdc_acm_line_coding_t line_coding = {0};
    coding_err = cdc_acm_host_line_coding_get(handle, &line_coding);
    if (coding_err == ESP_OK) {
        ESP_LOGI(TAG,
                 "CDC line coding: rate=%lu stop=%u parity=%u data=%u",
                 (unsigned long)line_coding.dwDTERate,
                 line_coding.bCharFormat, line_coding.bParityType,
                 line_coding.bDataBits);
    } else {
        ESP_LOGW(TAG, "CDC line coding read failed: %s",
                 esp_err_to_name(coding_err));
    }
    if (s_gone_handle == handle) {
        atomic_exchange(&s_gone_handle, NULL);
        (void)cdc_acm_host_close(handle);
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    s_cdc_handle = handle;
    s_cdc_ready = true;
    app_state_lock();
    app_state_mutable()->stm32_usb_connected = true;
    app_state_unlock();
    if (s_gone_handle == handle) {
        s_cdc_ready = false;
        clear_link_telemetry();
    }
    xSemaphoreGive(s_tx_mutex);
    ESP_LOGI(TAG, "C30D CDC link open, requested 115200 8N1");
    return ESP_OK;
}

static void clear_link_telemetry(void)
{
    app_state_lock();
    td_status_t *status = app_state_mutable();
    status->stm32_usb_connected = false;
    status->stm32_last_rx_us = 0;
    status->stm32_chassis_rx_us = 0;
    status->stm32_imu_rx_us = 0;
    status->stm32_sensor_rx_us = 0;
    status->stm32_safety_rx_us = 0;
    status->stm32_line_rx_us = 0;
    status->stm32_motor_rx_us = 0;
    status->motor.pan_flags = 0;
    app_state_unlock();
}

static void close_cdc_device(cdc_acm_dev_hdl_t expected_handle)
{
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    cdc_acm_dev_hdl_t handle = s_cdc_handle;
    if (handle == NULL || (expected_handle != NULL && expected_handle != handle)) {
        xSemaphoreGive(s_tx_mutex);
        return;
    }
    s_cdc_ready = false;
    s_cdc_handle = NULL;
    xSemaphoreGive(s_tx_mutex);

    (void)cdc_acm_host_close(handle);
    rp_parser_init(&s_parser);
    s_raw_rx_seen = false;
    clear_link_telemetry();
    if (s_params_rx != NULL) {
        xQueueReset(s_params_rx);
    }
    ESP_LOGW(TAG, "C30D USB CDC disconnected");
}

static void usb_link_task(void *arg)
{
    (void)arg;
    TickType_t last_diagnostic = xTaskGetTickCount();
    const cdc_acm_host_driver_config_t driver_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = USB_CDC_TASK_PRIORITY,
        .xCoreID = 0,
        .new_dev_cb = usb_new_device_callback,
    };

    esp_err_t err = bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV,
                                       true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB host start failed: %s", esp_err_to_name(err));
        xSemaphoreGive(s_usb_installed);
        vTaskDelete(NULL);
        return;
    }
    err = cdc_acm_host_install(&driver_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cdc_acm_host_install failed: %s",
                 esp_err_to_name(err));
        (void)bsp_usb_host_stop();
        xSemaphoreGive(s_usb_installed);
        vTaskDelete(NULL);
        return;
    }
    s_link_started = true;
    xSemaphoreGive(s_usb_installed);

    while (true) {
        usb_link_message_t message;
        bool got_message =
            xQueueReceive(s_usb_queue, &message, pdMS_TO_TICKS(500)) ==
            pdTRUE;
        TickType_t now = xTaskGetTickCount();
        if ((uint32_t)(now - last_diagnostic) >= pdMS_TO_TICKS(5000)) {
            td_status_t snap;
            app_state_snapshot(&snap);
            ESP_LOGI(TAG,
                     "CDC RX bytes=%lu frames=%lu crc=%lu len=%lu "
                     "connected=%d",
                     (unsigned long)snap.stm32_rx_bytes,
                     (unsigned long)snap.stm32_rx_frames,
                     (unsigned long)snap.stm32_crc_errors,
                     (unsigned long)snap.stm32_length_errors,
                     snap.stm32_usb_connected);
            last_diagnostic = now;
        }
        cdc_acm_dev_hdl_t gone = atomic_exchange(&s_gone_handle, NULL);
        if (gone != NULL) {
            close_cdc_device(gone);
        }
        if (got_message && message.type == USB_LINK_CONNECTED && s_cdc_handle == NULL) {
            (void)open_cdc_device(message.vid, message.pid);
        }
    }
}

static bool send_frame_with_timeout(uint8_t cmd,
                                    const uint8_t *payload,
                                    uint8_t length,
                                    TickType_t lock_timeout)
{
    uint8_t frame[RP_MAX_FRAME];
    size_t frame_length =
        rp_encode_frame(cmd, payload, length, frame, sizeof(frame));
    if (frame_length == 0u || s_tx_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_tx_mutex, lock_timeout) != pdTRUE) {
        return false;
    }
    bool ok = s_cdc_ready && s_cdc_handle != NULL &&
        cdc_acm_host_data_tx_blocking(
            s_cdc_handle, frame, frame_length,
            USB_CDC_TX_TIMEOUT_MS) == ESP_OK;
    xSemaphoreGive(s_tx_mutex);
    return ok;
}

static bool send_frame(uint8_t cmd, const uint8_t *payload, uint8_t length)
{
    return send_frame_with_timeout(
        cmd, payload, length, pdMS_TO_TICKS(100));
}

static bool send_frame_priority(uint8_t cmd,
                                const uint8_t *payload,
                                uint8_t length)
{
    return send_frame_with_timeout(
        cmd, payload, length,
        pdMS_TO_TICKS(USB_PRIORITY_LOCK_TIMEOUT_MS));
}

static bool send_frame_wait_ack(uint8_t cmd,
                                const uint8_t *payload,
                                uint8_t length)
{
    bool ok = false;
    TickType_t deadline;

    if (s_command_mutex == NULL || s_ack_semaphore == NULL ||
        xSemaphoreTake(s_command_mutex,
                       pdMS_TO_TICKS(USB_COMMAND_ACK_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }
    while (xSemaphoreTake(s_ack_semaphore, 0) == pdTRUE) {
    }
    set_ack_expected(cmd);
    if (send_frame(cmd, payload, length)) {
        deadline = xTaskGetTickCount() +
                   pdMS_TO_TICKS(USB_COMMAND_ACK_TIMEOUT_MS);
        for (;;) {
            int32_t remaining = (int32_t)(deadline - xTaskGetTickCount());
            if (remaining <= 0 ||
                xSemaphoreTake(s_ack_semaphore, (TickType_t)remaining) != pdTRUE) {
                break;
            }
            rp_ack_latch_t result = ack_snapshot();
            if (result.command != cmd || result.received) {
                ok = result.command == cmd && result.received && result.status == 0u;
                break;
            }
        }
    }
    set_ack_expected(0u);
    xSemaphoreGive(s_command_mutex);
    return ok;
}

static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t length)
{
    rp_chassis_state_t chassis;
    rp_sensor_state_t sensor;
    rp_safety_state_t safety;
    rp_line_state_t line_state;
    rp_imu_state_t imu;
    rp_motor_state_t motor;
    rp_command_ack_t ack;
    bool valid = false;
    if (cmd == RP_CMD_CHASSIS_STATE) {
        valid = rp_unpack_chassis_state(payload, length, &chassis);
    } else if (cmd == RP_CMD_SENSOR_STATE) {
        valid = rp_unpack_sensor_state(payload, length, &sensor);
    } else if (cmd == RP_CMD_SAFETY_STATE) {
        valid = rp_unpack_safety_state(payload, length, &safety);
    } else if (cmd == RP_CMD_LINE_STATE) {
        valid = rp_unpack_line_state(payload, length, &line_state);
    } else if (cmd == RP_CMD_IMU_STATE) {
        valid = rp_unpack_imu_state(payload, length, &imu);
    } else if (cmd == RP_CMD_MOTOR_STATE) {
        valid = rp_unpack_motor_state(payload, length, &motor);
    } else if (cmd == RP_CMD_PARAM_STATE) {
        uint8_t param_id;
        int32_t param_value;
        if (rp_unpack_param(payload, length, &param_id, &param_value)) {
            param_frame_t frame;
            memcpy(frame.payload, payload, sizeof(frame.payload));
            (void)xQueueSend(s_params_rx, &frame, 0);
        } else {
            ESP_LOGW(TAG, "invalid PARAM_STATE len=%u", length);
        }
        return;
    } else if (cmd == RP_CMD_COMMAND_ACK) {
        valid = rp_unpack_command_ack(payload, length, &ack);
        if (valid) {
            portENTER_CRITICAL(&s_ack_lock);
            bool matched = rp_ack_offer(&s_ack, &ack);
            portEXIT_CRITICAL(&s_ack_lock);
            if (matched) {
                xSemaphoreGive(s_ack_semaphore);
            }
        }
        return;
    } else {
        ESP_LOGW(TAG, "unexpected RX frame cmd=0x%02X", cmd);
        return;
    }
    if (!valid) {
        app_state_lock();
        uint32_t rejected = ++app_state_mutable()->stm32_payload_errors;
        app_state_unlock();
        if (rejected % 32u == 1u) {
            ESP_LOGW(TAG, "invalid C30D payload cmd=0x%02X len=%u, rejected=%lu",
                     cmd, length, (unsigned long)rejected);
        }
        return;
    }

    app_state_lock();
    td_status_t *status = app_state_mutable();
    bool first_frame = !status->stm32_ever_rx;
    status->stm32_ever_rx = true;
    status->stm32_usb_connected = true;
    status->stm32_last_rx_us = app_now_us();
    status->stm32_last_cmd = cmd;
    ++status->stm32_rx_frames;
    if (cmd == RP_CMD_CHASSIS_STATE) {
        ++status->stm32_chassis_frames;
        status->chassis = chassis;
        status->stm32_chassis_rx_us = status->stm32_last_rx_us;
    } else if (cmd == RP_CMD_SENSOR_STATE) {
        ++status->stm32_sensor_frames;
        status->sensor = sensor;
        status->stm32_sensor_rx_us = status->stm32_last_rx_us;
    } else if (cmd == RP_CMD_SAFETY_STATE) {
        ++status->stm32_safety_frames;
        status->safety = safety;
        status->stm32_safety_rx_us = status->stm32_last_rx_us;
    } else if (cmd == RP_CMD_LINE_STATE) {
        ++status->stm32_line_frames;
        status->line_state = line_state;
        status->stm32_line_rx_us = status->stm32_last_rx_us;
    } else if (cmd == RP_CMD_IMU_STATE) {
        ++status->stm32_imu_frames;
        status->imu = imu;
        status->stm32_imu_rx_us = status->stm32_last_rx_us;
    } else {
        ++status->stm32_motor_frames;
        status->motor = motor;
        status->stm32_motor_rx_us = status->stm32_last_rx_us;
    }
    app_state_unlock();

    if (first_frame) {
        ESP_LOGI(TAG, "first valid C30D frame cmd=0x%02X", cmd);
    }
}

void robot_link_publish_telemetry(void (*param_received)(uint8_t id, int32_t value))
{
    static uint32_t last_count[6];
    td_status_t status;
    app_state_snapshot(&status);
    if (!status.stm32_online) {
        return;
    }
    const struct {
        const char *topic;
        const void *value;
        size_t bytes;
        uint32_t count;
        int64_t rx_us;
    } samples[] = {
        {"chassis_state", &status.chassis, sizeof(status.chassis), status.stm32_chassis_frames, status.stm32_chassis_rx_us},
        {"sensor_state", &status.sensor, sizeof(status.sensor), status.stm32_sensor_frames, status.stm32_sensor_rx_us},
        {"safety_state", &status.safety, sizeof(status.safety), status.stm32_safety_frames, status.stm32_safety_rx_us},
        {"line_state", &status.line_state, sizeof(status.line_state), status.stm32_line_frames,
         status.stm32_line_rx_us},
        {"imu_state", &status.imu, sizeof(status.imu), status.stm32_imu_frames, status.stm32_imu_rx_us},
        {"motor_state", &status.motor, sizeof(status.motor), status.stm32_motor_frames, status.stm32_motor_rx_us},
    };
    /* The CDC callback must be free to receive ACKs while a bus service waits. */
    int64_t now_us = app_now_us();
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        if (app_rx_age_ms(now_us, samples[i].rx_us) < TD_STM32_ONLINE_MS &&
            samples[i].count != last_count[i]) {
            last_count[i] = samples[i].count;
            publish_topic(samples[i].topic, samples[i].value, samples[i].bytes);
        }
    }

    param_frame_t frame;
    for (unsigned i = 0; i < 8 && s_params_rx != NULL &&
         xQueueReceive(s_params_rx, &frame, 0) == pdTRUE; ++i) {
        uint8_t id;
        int32_t value;
        if (rp_unpack_param(frame.payload, sizeof(frame.payload), &id, &value)) {
            if (param_received != NULL) {
                param_received(id, value);
            }
            publish_topic("param_state", frame.payload, sizeof(frame.payload));
        }
    }
}

esp_err_t robot_link_init(void)
{
    s_tx_mutex = xSemaphoreCreateMutex();
    s_command_mutex = xSemaphoreCreateMutex();
    s_ack_semaphore = xSemaphoreCreateBinary();
    s_usb_installed = xSemaphoreCreateBinary();
    s_usb_queue = xQueueCreate(8, sizeof(usb_link_message_t));
    s_params_rx = xQueueCreate(8, sizeof(param_frame_t));
    if (s_tx_mutex == NULL || s_command_mutex == NULL ||
        s_ack_semaphore == NULL || s_usb_installed == NULL ||
        s_usb_queue == NULL || s_params_rx == NULL) {
        return ESP_ERR_NO_MEM;
    }

    rp_parser_init(&s_parser);
    s_cdc_handle = NULL;
    s_cdc_ready = false;
    s_link_started = false;
    s_raw_rx_seen = false;
    s_gone_handle = NULL;
    set_ack_expected(0u);
    if (xTaskCreate(usb_link_task, "usb_chassis", USB_LINK_TASK_STACK,
                    NULL, USB_LINK_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_usb_installed, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return s_link_started ? ESP_OK : ESP_FAIL;
}

bool robot_link_send_drive(int8_t left_percent, int8_t right_percent)
{
    rp_drive_cmd_t command = {left_percent, right_percent};
    uint8_t payload[RP_DRIVE_LEN];
    return rp_pack_drive(&command, payload) &&
           send_frame_wait_ack(RP_CMD_DRIVE, payload, sizeof(payload));
}

bool robot_link_send_line_mode(bool enable, uint8_t speed_percent)
{
    rp_line_cmd_t command = {
        .enable = enable ? 1u : 0u,
        .speed_percent = speed_percent,
    };
    uint8_t payload[RP_LINE_LEN + 1u];
    if (!rp_pack_line_mode(&command, payload)) {
        return false;
    }
    uint8_t length = RP_LINE_LEN;
#if TD_LINE_ASSIST_ENABLED && TD_LINE_MODEL_AVAILABLE
    if (enable && speed_percent <= 15u) {
        td_status_t status;
        app_state_snapshot(&status);
        const td_line_sample_t *sample = &status.line_learning;
        if (sample->valid && status.mode == TD_MODE_LINE &&
            app_rx_age_ms(app_now_us(), sample->frame_us) <= 180u) {
            int current = ((int)sample->ccd_center - 64) * 100 / 64;
            int bias = (sample->prediction - current) / 16;
            if (bias < -5) bias = -5;
            if (bias > 5) bias = 5;
            if (bias != 0) {
                payload[length++] = (uint8_t)(int8_t)bias;
            }
        }
    }
#endif
    return send_frame_wait_ack(RP_CMD_LINE, payload, length);
}

bool robot_link_send_actuator(uint16_t pan_deg, uint8_t push_action)
{
    rp_actuator_cmd_t command = {pan_deg, push_action};
    uint8_t payload[RP_ACTUATOR_LEN];
    return rp_pack_actuator(&command, payload) &&
           send_frame_wait_ack(RP_CMD_ACTUATOR, payload, sizeof(payload));
}

bool robot_link_send_stop_all(void)
{
    return send_frame_wait_ack(RP_CMD_STOP_ALL, NULL, 0);
}

bool robot_link_send_calibrate_pan(void)
{
    return send_frame_wait_ack(RP_CMD_CALIBRATE_PAN, NULL, 0);
}

bool robot_link_send_set_param(uint8_t id, int32_t value)
{
    uint8_t payload[RP_PARAM_FRAME_LEN];
    return rp_pack_param(id, value, payload) &&
           send_frame_wait_ack(RP_CMD_SET_PARAM, payload, sizeof(payload));
}

bool robot_link_emergency_stop(void)
{
    return send_frame_priority(RP_CMD_ESTOP, NULL, 0);
}

bool robot_link_clear_estop(void)
{
    return send_frame_wait_ack(RP_CMD_CLEAR_ESTOP, NULL, 0);
}
