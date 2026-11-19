#pragma once

#include <stddef.h>
#include <stdint.h>

#define TD_C30D_HISTORY_CAPACITY 60u
#define TD_C30D_JSON_CAPACITY    24576u

typedef struct {
    uint32_t uptime_ms;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    int16_t yaw_cdeg;
    uint16_t distance_cm;
    uint16_t battery_mv;
    uint8_t robot_state;
    uint8_t fault_flags;
    uint8_t online;
    uint8_t line_flags;
    uint8_t line_source;
    uint8_t ccd_center;
    uint8_t ccd_confidence;
    int8_t line_error;
    uint8_t rewind_progress;
    int16_t gyro_x_ddps;
    int16_t gyro_y_ddps;
    int16_t gyro_z_ddps;
    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;
} td_c30d_sample_t;

typedef struct {
    uint32_t last_rx_age_ms;
    uint32_t payload_errors;
    uint32_t chassis_age_ms;
    uint32_t sensor_age_ms;
    uint32_t safety_age_ms;
    uint32_t line_age_ms;
    uint32_t imu_age_ms;
    uint32_t motor_age_ms;
    uint32_t rx_frames;
    uint32_t rx_bytes;
    uint32_t chassis_frames;
    uint32_t sensor_frames;
    uint32_t safety_frames;
    uint32_t line_frames;
    uint32_t imu_frames;
    uint32_t motor_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    int16_t yaw_cdeg;
    uint16_t distance_cm;
    uint16_t battery_mv;
    uint16_t c30d_uptime_s;
    uint16_t last_cmd_age_ms;
    uint8_t usb_connected;
    uint8_t online;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint8_t robot_state;
    uint8_t fault_flags;
    uint8_t last_cmd;
    uint8_t line_flags;
    uint8_t line_source;
    uint8_t ccd_center;
    uint8_t ccd_confidence;
    int8_t line_error;
    uint8_t rewind_progress;
    int16_t gyro_x_ddps;
    int16_t gyro_y_ddps;
    int16_t gyro_z_ddps;
    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;
    int16_t encoder_delta_a;
    int16_t encoder_delta_b;
    int16_t pwm_a;
    int16_t pwm_b;
    int16_t target_a_mm_s;
    int16_t target_b_mm_s;
    uint16_t pan_deg;
    uint8_t control_source;
    int8_t left_command;
    int8_t right_command;
} td_c30d_meta_t;

/*
 * The monitor is a pure in-memory ring. Callers serialize access with
 * app_state_lock(); this keeps the host unit tests free of RTOS dependencies.
 */
void td_c30d_monitor_init(void);
void td_c30d_monitor_poll(const td_c30d_sample_t *sample, uint32_t now_ms);
size_t td_c30d_monitor_snapshot(td_c30d_sample_t *out, size_t capacity);
size_t td_c30d_monitor_export_csv(char *out, size_t capacity);
size_t td_c30d_monitor_export_json(const td_c30d_meta_t *meta,
                                   const td_c30d_sample_t *samples,
                                   size_t count,
                                   char *out,
                                   size_t capacity);
