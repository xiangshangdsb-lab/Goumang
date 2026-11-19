#include <stdio.h>

#include "agent/agent_policy.h"

static int checks;
static int failures;

#define CHECK(value)                                                     \
    do {                                                                 \
        ++checks;                                                        \
        if (!(value)) {                                                  \
            ++failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);      \
        }                                                                \
    } while (0)

int main(void)
{
    CHECK(agent_policy_valid_api_key("sk-unit-test"));
    CHECK(!agent_policy_valid_api_key(NULL));
    CHECK(!agent_policy_valid_api_key("short"));
    CHECK(!agent_policy_valid_api_key("ab-123456789"));
    CHECK(agent_policy_manual_move(-40, 40, 100));
    CHECK(agent_policy_manual_move(40, -40, 1500));
    CHECK(!agent_policy_manual_move(-41, 0, 500));
    CHECK(!agent_policy_manual_move(0, 41, 500));
    CHECK(!agent_policy_manual_move(0, 0, 99));
    CHECK(!agent_policy_manual_move(0, 0, 1501));
    CHECK(agent_policy_line_speed(5));
    CHECK(agent_policy_line_speed(40));
    CHECK(!agent_policy_line_speed(4));
    CHECK(!agent_policy_line_speed(41));
    CHECK(agent_policy_pan_angle(0));
    CHECK(agent_policy_pan_angle(540));
    CHECK(!agent_policy_pan_angle(-1));
    CHECK(!agent_policy_pan_angle(541));
    CHECK(agent_policy_tool_class("get_robot_status") ==
          AGENT_TOOL_READ);
    CHECK(agent_policy_tool_class("emergency_stop") ==
          AGENT_TOOL_SAFETY);
    CHECK(agent_policy_tool_class("stop_robot") ==
          AGENT_TOOL_SAFETY);
    CHECK(agent_policy_tool_class("manual_move") ==
          AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_class("start_line_follow") ==
          AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_class("set_pan") ==
          AGENT_TOOL_CONTROL);
    CHECK(agent_policy_tool_class("start_inspection") ==
          AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_class("submit_plan") == AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_class("set_control_param") == AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_class("capture_photo") == AGENT_TOOL_READ);
    CHECK(agent_policy_field_action("FOLLOW_LINE") == FIELD_FOLLOW_LINE);
    CHECK(agent_policy_field_action("STOP") == FIELD_STOP);
    CHECK(agent_policy_field_action("BACKWARD_SHORT") == FIELD_BACKWARD_SHORT);
    CHECK(agent_policy_field_action("drive") == FIELD_INVALID);
    CHECK(agent_policy_field_action(NULL) == FIELD_INVALID);
    CHECK(agent_policy_tool_class("clear_emergency_stop") ==
          AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_class("unknown") == AGENT_TOOL_DENIED);
    CHECK(agent_policy_tool_allowed("get_robot_status", 1u));
    CHECK(agent_policy_tool_allowed("emergency_stop", 1u));
    CHECK(agent_policy_tool_allowed("field_decision", 0u));
    CHECK(!agent_policy_tool_allowed("field_decision", 1u));
    CHECK(!agent_policy_tool_allowed("clear_emergency_stop", 0u));
    printf("checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
