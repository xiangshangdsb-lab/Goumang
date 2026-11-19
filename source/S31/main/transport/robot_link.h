#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t robot_link_init(void);
void robot_link_publish_telemetry(void (*param_received)(uint8_t id, int32_t value));
bool robot_link_send_drive(int8_t left_percent, int8_t right_percent);
bool robot_link_send_line_mode(bool enable, uint8_t speed_percent);
bool robot_link_send_actuator(uint16_t pan_deg, uint8_t push_action);
bool robot_link_send_stop_all(void);
bool robot_link_send_calibrate_pan(void);
bool robot_link_send_set_param(uint8_t id, int32_t value);
bool robot_link_emergency_stop(void);
bool robot_link_clear_estop(void);
