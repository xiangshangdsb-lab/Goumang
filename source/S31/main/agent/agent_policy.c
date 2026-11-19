#include "agent_policy.h"
#include "protocol/robot_protocol.h"

#include <string.h>

bool agent_policy_valid_api_key(const char *key)
{
    size_t length;
    if (key == NULL) {
        return false;
    }
    length = strlen(key);
    return length >= 8u && length < 256u &&
           key[0] == 's' && key[1] == 'k' && key[2] == '-';
}

bool agent_policy_manual_move(int left, int right, int hold_ms)
{
    return left >= -40 && left <= 40 &&
           right >= -40 && right <= 40 &&
           hold_ms >= 100 && hold_ms <= 1500;
}

bool agent_policy_line_speed(int speed_percent)
{
    return speed_percent >= 5 && speed_percent <= 40;
}

bool agent_policy_pan_angle(int angle_deg)
{
    return angle_deg >= 0 && angle_deg <= RP_PAN_MAX_DEG;
}

field_action_t agent_policy_field_action(const char *name)
{
    static const char *const names[] = {
        "", "STOP", "FOLLOW_LINE", "FORWARD_SHORT", "BACKWARD_SHORT",
        "TURN_LEFT_SMALL", "TURN_RIGHT_SMALL",
    };
    if (name != NULL) {
        for (unsigned i = FIELD_STOP; i < sizeof(names) / sizeof(names[0]); ++i) {
            if (strcmp(name, names[i]) == 0) {
                return (field_action_t)i;
            }
        }
    }
    return FIELD_INVALID;
}

agent_tool_class_t agent_policy_tool_class(const char *name)
{
    if (name == NULL) {
        return AGENT_TOOL_DENIED;
    }
    if (strcmp(name, "get_robot_status") == 0 ||
        strcmp(name, "get_telemetry") == 0 ||
        strcmp(name, "get_line_state") == 0 ||
        strcmp(name, "get_vision_summary") == 0 ||
        strcmp(name, "capture_photo") == 0 ||
        strcmp(name, "get_takeover_status") == 0) {
        return AGENT_TOOL_READ;
    }
    if (strcmp(name, "stop_robot") == 0 ||
        strcmp(name, "emergency_stop") == 0) {
        return AGENT_TOOL_SAFETY;
    }
    if (strcmp(name, "field_decision") == 0 ||
        strcmp(name, "set_pan") == 0) {
        return AGENT_TOOL_CONTROL;
    }
    return AGENT_TOOL_DENIED;
}

bool agent_policy_tool_allowed(const char *name,
                               uint8_t control_actions_used)
{
    agent_tool_class_t tool_class = agent_policy_tool_class(name);

    if (tool_class == AGENT_TOOL_READ || tool_class == AGENT_TOOL_SAFETY) {
        return true;
    }
    return tool_class == AGENT_TOOL_CONTROL && control_actions_used == 0u;
}
