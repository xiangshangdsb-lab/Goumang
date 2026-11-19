#include "app_state.h"

#include <string.h>

#include "app_config.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static td_status_t s_status;
static SemaphoreHandle_t s_mutex;

void app_state_init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.threshold = 90;
    s_status.mode = TD_MODE_IDLE;
    s_status.pan_deg = RP_PAN_HOME_DEG;
    s_status.sensor.distance_cm = 0xFFFFu;
    strlcpy(s_status.ap_ip, "192.168.4.1", sizeof(s_status.ap_ip));
    strlcpy(s_status.url, TD_AP_URL, sizeof(s_status.url));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

void app_state_lock(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
}

void app_state_unlock(void)
{
    xSemaphoreGive(s_mutex);
}

td_status_t *app_state_mutable(void)
{
    return &s_status;
}

void app_state_snapshot(td_status_t *out)
{
    if (out == NULL) {
        return;
    }
    app_state_lock();
    *out = s_status;
    app_state_unlock();

    out->stm32_online =
        out->stm32_usb_connected &&
        app_rx_age_ms(app_now_us(), out->stm32_last_rx_us) < TD_STM32_ONLINE_MS;
}

void app_state_build_c30d_meta(const td_status_t *status,
                               td_c30d_meta_t *meta)
{
    if (status == NULL || meta == NULL) {
        return;
    }
    memset(meta, 0, sizeof(*meta));
    meta->usb_connected = status->stm32_usb_connected ? 1u : 0u;
    meta->online = status->stm32_online ? 1u : 0u;
    meta->rx_frames = status->stm32_rx_frames;
    meta->rx_bytes = status->stm32_rx_bytes;
    meta->chassis_frames = status->stm32_chassis_frames;
    meta->sensor_frames = status->stm32_sensor_frames;
    meta->safety_frames = status->stm32_safety_frames;
    meta->line_frames = status->stm32_line_frames;
    meta->imu_frames = status->stm32_imu_frames;
    meta->motor_frames = status->stm32_motor_frames;
    meta->crc_errors = status->stm32_crc_errors;
    meta->length_errors = status->stm32_length_errors;
    meta->last_cmd = status->stm32_last_cmd;
    meta->left_speed_mm_s = status->chassis.left_speed_mm_s;
    meta->right_speed_mm_s = status->chassis.right_speed_mm_s;
    meta->yaw_cdeg = status->chassis.yaw_cdeg;
    meta->distance_cm = status->sensor.distance_cm;
    meta->battery_mv = status->sensor.battery_mv;
    meta->robot_state = status->safety.robot_state;
    meta->fault_flags = status->safety.fault_flags;
    meta->fw_major = status->safety.fw_major;
    meta->fw_minor = status->safety.fw_minor;
    meta->c30d_uptime_s = status->safety.uptime_s;
    meta->last_cmd_age_ms = status->safety.last_cmd_age_ms;
    meta->line_flags = status->line_state.flags;
    meta->line_source = status->line_state.source;
    meta->ccd_center = status->line_state.ccd_center;
    meta->ccd_confidence = status->line_state.ccd_confidence;
    meta->line_error = status->line_state.line_error;
    meta->rewind_progress = status->line_state.rewind_progress;
    meta->gyro_x_ddps = status->imu.gyro_x_ddps;
    meta->gyro_y_ddps = status->imu.gyro_y_ddps;
    meta->gyro_z_ddps = status->imu.gyro_z_ddps;
    meta->accel_x_mg = status->imu.accel_x_mg;
    meta->accel_y_mg = status->imu.accel_y_mg;
    meta->accel_z_mg = status->imu.accel_z_mg;
    meta->encoder_delta_a = status->motor.encoder_delta_a;
    meta->encoder_delta_b = status->motor.encoder_delta_b;
    meta->pwm_a = status->motor.pwm_a;
    meta->pwm_b = status->motor.pwm_b;
    meta->target_a_mm_s = status->motor.target_a_mm_s;
    meta->target_b_mm_s = status->motor.target_b_mm_s;
    meta->pan_deg = status->motor.pan_deg;
    meta->control_source = status->motor.control_source;
    meta->left_command = status->motor.left_command;
    meta->right_command = status->motor.right_command;

    int64_t now_us = app_now_us();
    meta->last_rx_age_ms = app_rx_age_ms(now_us, status->stm32_last_rx_us);
    meta->payload_errors = status->stm32_payload_errors;
    meta->chassis_age_ms = app_rx_age_ms(now_us, status->stm32_chassis_rx_us);
    meta->sensor_age_ms = app_rx_age_ms(now_us, status->stm32_sensor_rx_us);
    meta->safety_age_ms = app_rx_age_ms(now_us, status->stm32_safety_rx_us);
    meta->line_age_ms = app_rx_age_ms(now_us, status->stm32_line_rx_us);
    meta->imu_age_ms = app_rx_age_ms(now_us, status->stm32_imu_rx_us);
    meta->motor_age_ms = app_rx_age_ms(now_us, status->stm32_motor_rx_us);
}

int64_t app_now_us(void)
{
    return esp_timer_get_time();
}

uint32_t app_now_ms(void)
{
    return (uint32_t)(app_now_us() / 1000);
}

const char *td_mode_name(td_mode_t mode)
{
    switch (mode) {
    case TD_MODE_LINE:
        return "LINE";
    case TD_MODE_MANUAL:
        return "MANUAL";
    case TD_MODE_ESTOP:
        return "ESTOP";
    case TD_MODE_INSPECT:
        return "INSPECT";
    case TD_MODE_IDLE:
    default:
        return "IDLE";
    }
}
