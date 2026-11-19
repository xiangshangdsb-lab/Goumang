#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AGENT_TOOL_DENIED = 0,
    AGENT_TOOL_READ,
    AGENT_TOOL_SAFETY,
    AGENT_TOOL_CONTROL,
} agent_tool_class_t;

typedef enum {
    FIELD_INVALID = 0,
    FIELD_STOP,
    FIELD_FOLLOW_LINE,
    FIELD_FORWARD_SHORT,
    FIELD_BACKWARD_SHORT,
    FIELD_TURN_LEFT_SMALL,
    FIELD_TURN_RIGHT_SMALL,
} field_action_t;

field_action_t agent_policy_field_action(const char *name);
bool agent_policy_valid_api_key(const char *key);
bool agent_policy_manual_move(int left, int right, int hold_ms);
bool agent_policy_line_speed(int speed_percent);
bool agent_policy_pan_angle(int angle_deg);
agent_tool_class_t agent_policy_tool_class(const char *name);
bool agent_policy_tool_allowed(const char *name,
                               uint8_t control_actions_used);

