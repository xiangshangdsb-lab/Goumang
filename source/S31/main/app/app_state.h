#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "diagnostics/c30d_monitor.h"
#include "control/line_follower.h"
#include "protocol/robot_protocol.h"
#include "perception/vision.h"
#include "perception/line_learning.h"

typedef enum {
    TD_MODE_IDLE = 0,
    TD_MODE_LINE,
    TD_MODE_MANUAL,
    TD_MODE_ESTOP,
    TD_MODE_INSPECT,
} td_mode_t;

typedef struct {
    bool display_ready;
    bool touch_ready;
    bool camera_ready;
    bool camera_streaming;
    bool vision_ok;
    bool web_ready;
    bool ap_started;
    bool ap_client_connected;
    bool sta_connected;
    bool setup_ap;
    uint8_t wifi_preferred_mode;

    uint8_t threshold;
    uint32_t vision_frames;
    uint16_t vision_fps_x10;
    td_vision_result_t vision;
    td_line_sample_t line_learning;
    line_follow_result_t line;

    td_mode_t mode;
    int8_t out_left;
    int8_t out_right;
    uint16_t pan_deg;
    uint8_t push_action;
    uint8_t inspect_phase;

    bool stm32_ever_rx;
    bool stm32_usb_connected;
    bool stm32_online;
    int64_t stm32_last_rx_us;
    int64_t stm32_chassis_rx_us;
    int64_t stm32_imu_rx_us;
    int64_t stm32_sensor_rx_us;
    int64_t stm32_safety_rx_us;
    int64_t stm32_motor_rx_us;
    int64_t stm32_line_rx_us;
    uint32_t stm32_rx_frames;
    uint32_t stm32_rx_bytes;
    uint32_t stm32_chassis_frames;
    uint32_t stm32_sensor_frames;
    uint32_t stm32_safety_frames;
    uint32_t stm32_line_frames;
    uint32_t stm32_imu_frames;
    uint32_t stm32_motor_frames;
    uint32_t stm32_crc_errors;
    uint32_t stm32_length_errors;
    uint32_t stm32_payload_errors;
    uint8_t stm32_last_cmd;
    rp_chassis_state_t chassis;
    rp_sensor_state_t sensor;
    rp_safety_state_t safety;
    rp_line_state_t line_state;
    rp_imu_state_t imu;
    rp_motor_state_t motor;

    char ap_ip[16];
    char url[32];
} td_status_t;

void app_state_init(void);
void app_state_lock(void);
void app_state_unlock(void);
td_status_t *app_state_mutable(void);
void app_state_snapshot(td_status_t *out);
void app_state_build_c30d_meta(const td_status_t *status,
                               td_c30d_meta_t *meta);

/* UINT32_MAX means no sample or a timestamp outside the usable range. */
static inline uint32_t app_rx_age_ms(int64_t now_us, int64_t rx_us)
{
    if (rx_us <= 0 || rx_us > now_us) {
        return UINT32_MAX;
    }
    int64_t age_ms = (now_us - rx_us) / 1000;
    return age_ms >= UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;
}

int64_t app_now_us(void);
uint32_t app_now_ms(void);
const char *td_mode_name(td_mode_t mode);
