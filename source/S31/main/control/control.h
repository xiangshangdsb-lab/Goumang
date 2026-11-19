#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "agent/agent_policy.h"

esp_err_t control_start(void);
void control_start_line(void);
bool control_start_line_speed(uint8_t speed_percent);
bool control_start_inspect(void);
bool control_stop(void);
bool control_halt(void);
const char *control_field_move(field_action_t action);
bool control_field_stationary(void);
void control_field_cancel(void);
bool control_manual(int8_t left, int8_t right, uint32_t hold_ms);
bool control_estop(void);
bool control_clear_estop(void);
bool control_actuator(uint16_t pan_deg, uint8_t push_action);
bool control_calibrate_pan(void);
bool control_set_param(uint8_t id, int32_t value);
bool control_get_param(uint8_t id, int32_t *value);
void control_param_state(uint8_t id, int32_t value);
