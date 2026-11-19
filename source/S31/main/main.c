/*
 * Goumang on ESP32-S31-Korvo-1.
 *
 * Boot order: display/UI -> voice -> AI services -> camera -> Wi-Fi/web ->
 * USB chassis link -> patrol state machine. Every optional hardware block
 * degrades to "not ready" instead of stopping the board.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "bsp/esp-bsp.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "mbedtls/platform.h"

#include "app/app_config.h"
#include "app/app_state.h"
#include "agent/agent_runtime.h"
#include "agent/agent_service.h"
#include "diagnostics/c30d_monitor.h"
#include "perception/camera_app.h"
#include "control/control.h"
#include "network/device_auth_store.h"
#include "perception/face_service.h"
#include "perception/object_service.h"
#include "bus/robot_bus.h"
#include "transport/robot_link.h"
#include "app/robot_services.h"
#include "interface/panel/ui.h"
#include "perception/vision.h"
#include "audio/voice_player.h"
#include "interface/web/web_server.h"
#include "network/wifi_mgr.h"
#include "diagnostics/startup_selftest.h"

static const char *TAG = "goumang";

static void *tls_psram_calloc(size_t count, size_t size)
{
    if (size != 0u && count > SIZE_MAX / size) return NULL;
    return heap_caps_calloc_prefer(count, size, 2,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static void capture_c30d_history(void)
{
    td_status_t status;
    td_c30d_sample_t sample = {0};

    app_state_snapshot(&status);
    sample.online = status.stm32_online ? 1u : 0u;
    sample.robot_state = status.safety.robot_state;
    sample.fault_flags = status.safety.fault_flags;
    sample.left_speed_mm_s = status.chassis.left_speed_mm_s;
    sample.right_speed_mm_s = status.chassis.right_speed_mm_s;
    sample.yaw_cdeg = status.chassis.yaw_cdeg;
    sample.distance_cm = status.sensor.distance_cm;
    sample.battery_mv = status.sensor.battery_mv;
    sample.line_flags = status.line_state.flags;
    sample.line_source = status.line_state.source;
    sample.ccd_center = status.line_state.ccd_center;
    sample.ccd_confidence = status.line_state.ccd_confidence;
    sample.line_error = status.line_state.line_error;
    sample.rewind_progress = status.line_state.rewind_progress;
    sample.gyro_x_ddps = status.imu.gyro_x_ddps;
    sample.gyro_y_ddps = status.imu.gyro_y_ddps;
    sample.gyro_z_ddps = status.imu.gyro_z_ddps;
    sample.accel_x_mg = status.imu.accel_x_mg;
    sample.accel_y_mg = status.imu.accel_y_mg;
    sample.accel_z_mg = status.imu.accel_z_mg;

    app_state_lock();
    td_c30d_monitor_poll(&sample, app_now_ms());
    app_state_unlock();
}

static void agent_runtime_task(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t now = app_now_ms();
        robot_link_publish_telemetry(control_param_state);
        rb_action_tick(now);
        agent_runtime_tick(now);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    while (true) {
        capture_c30d_history();
        ui_refresh();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    /* Model metadata includes many small C++ allocations. Keep general
     * allocations in PSRAM so stacks, DMA and lwIP retain internal RAM. */
    heap_caps_malloc_extmem_enable(0u);
    /* Install the supported mbedTLS allocator before any network service.
     * Vision and LCD allocations leave insufficient internal RAM for TLS
     * records; PSRAM is already initialized by ESP-IDF before app_main. */
    mbedtls_platform_set_calloc_free(tls_psram_calloc, heap_caps_free);
    esp_chip_info_t chip;
    esp_err_t err;
    startup_init_t init = {0};
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "Goumang S31 %s", TD_FW_VERSION);
    ESP_LOGI(TAG, "chip=esp32s31 cores=%d rev=%d.%d", (int)chip.cores,
             (int)(chip.revision / 100), (int)(chip.revision % 100));
    ESP_LOGI(TAG, "reset reason=%d", (int)esp_reset_reason());

    app_state_init();
    rb_init();
    robot_services_init();
    td_c30d_monitor_init();
    err = device_auth_store_init();
    init.storage = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "device auth NVS warning: %s",
                 esp_err_to_name(err));
    }

    lv_display_t *display = bsp_display_start();
    if (display == NULL) {
        ESP_LOGE(TAG, "LCD initialization failed");
    } else {
        bsp_display_backlight_on();
        ui_start();
    }

    init.display = display != NULL;
    init.touch = display != NULL && bsp_display_get_input_dev() != NULL;
    app_state_lock();
    app_state_mutable()->display_ready = (display != NULL);
    app_state_mutable()->touch_ready = init.touch;
    app_state_unlock();

    /* The S31 ADC needs the BSP's on-chip bit-weight calibration before
     * the resistor-ladder buttons can be mapped to reliable millivolts. */
    button_handle_t buttons[BSP_BUTTON_NUM] = {0};
    int button_count = 0;
    err = bsp_iot_button_create(buttons, &button_count, BSP_BUTTON_NUM);
    init.keys = err == ESP_OK && button_count == BSP_BUTTON_NUM;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC button initialization failed: %s",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "ADC buttons ready: %d", button_count);
    }

    /* Black-line detector is pure CPU; it runs even without a camera. */
    err = vision_start();
    app_state_lock();
    app_state_mutable()->vision_ok = (err == ESP_OK);
    app_state_unlock();

    /* Chinese voice prompts from the "voice" SPIFFS partition. */
    err = voice_init();
    if (err == ESP_OK) {
        voice_play_async(TD_VOICE_DEFAULT_CLIP);
    } else {
        ESP_LOGE(TAG, "voice init failed: %s", esp_err_to_name(err));
    }

    if (display != NULL && xTaskCreatePinnedToCore(ui_task, "ui_refresh", 12288, NULL, 3,
                                NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "UI refresh task creation failed");
    }

    err = face_service_start();
    init.face = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "face service startup failed: %s",
                 esp_err_to_name(err));
    }
    err = object_service_start();
    init.objects = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "object service startup failed: %s",
                 esp_err_to_name(err));
    }

    /* DVP camera (OV3660 or SC101IOT); failure is non-fatal. */
    err = display != NULL ? camera_app_start() : ESP_ERR_INVALID_STATE;
    init.camera = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera start failed: %s", esp_err_to_name(err));
        app_state_lock();
        app_state_mutable()->camera_ready = false;
        app_state_mutable()->camera_streaming = false;
        app_state_unlock();
    }

    /* Native Wi-Fi AP + web console. */
    err = wifi_mgr_start();
    init.wifi = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi start failed: %s", esp_err_to_name(err));
    }

    /* USB-A host owns GPIO19/20 (SD_D0 shares GPIO20), so this build keeps
     * all required assets in flash and does not mount a microSD card. */
    err = robot_link_init();
    init.usb = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "C30D USB link startup failed: %s",
                 esp_err_to_name(err));
    }
    err = control_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "patrol control startup failed: %s",
                 esp_err_to_name(err));
    }
    agent_runtime_init();
    if (xTaskCreate(agent_runtime_task, "agent_runtime", 6144, NULL, 4,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "agent runtime task creation failed");
    }
    err = agent_service_start();
    init.agent = err == ESP_OK;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DeepSeek agent startup failed: %s",
                 esp_err_to_name(err));
    }

    if (init.wifi && web_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "web server start failed");
    }

    td_status_t snap;
    app_state_snapshot(&snap);
    ESP_LOGI(TAG, "ready: URL=%s camera=%s voice=%s web=%s agent=%s",
             snap.url,
             snap.camera_ready ? "ok" : "missing",
             voice_ready() ? "ok" : "missing",
             snap.web_ready ? "ok" : "missing",
             agent_service_is_configured() ? "configured" : "setup");
    err = startup_selftest_start(&init);
    if (err != ESP_OK) ESP_LOGE(TAG, "startup self-test failed: %s", esp_err_to_name(err));
    ESP_LOGI(TAG, "heap: internal=%u largest=%u psram=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}
