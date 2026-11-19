#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "bluetooth/bluetooth_control.h"

static int s_checks;
static int s_failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        ++s_checks;                                                        \
        if (!(condition)) {                                                \
            ++s_failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
        }                                                                  \
    } while (0)

static void test_movement_commands(void)
{
    td_bt_command_t command;

    CHECK(td_bt_parse_command("A", &command));
    CHECK(command.action == TD_BT_ACTION_FORWARD);
    CHECK(command.left_percent == 35);
    CHECK(command.right_percent == 35);
    CHECK(!command.explicit_speed);

    CHECK(td_bt_parse_command("b60", &command));
    CHECK(command.action == TD_BT_ACTION_BACKWARD);
    CHECK(command.left_percent == -60);
    CHECK(command.right_percent == -60);
    CHECK(command.explicit_speed);

    CHECK(td_bt_parse_command("L30", &command));
    CHECK(command.action == TD_BT_ACTION_TURN_LEFT);
    CHECK(command.left_percent == -30);
    CHECK(command.right_percent == 30);

    CHECK(td_bt_parse_command("r25", &command));
    CHECK(command.action == TD_BT_ACTION_TURN_RIGHT);
    CHECK(command.left_percent == 25);
    CHECK(command.right_percent == -25);
}

static void test_wheeltec_app_commands(void)
{
    td_bt_command_t command;

    CHECK(td_bt_parse_command("E", &command));
    CHECK(command.action == TD_BT_ACTION_BACKWARD);
    CHECK(command.left_percent == -35);
    CHECK(command.right_percent == -35);

    CHECK(td_bt_parse_command("C", &command));
    CHECK(command.action == TD_BT_ACTION_TURN_RIGHT);
    CHECK(command.left_percent == 35);
    CHECK(command.right_percent == -35);

    CHECK(td_bt_parse_command("G", &command));
    CHECK(command.action == TD_BT_ACTION_TURN_LEFT);
    CHECK(command.left_percent == -35);
    CHECK(command.right_percent == 35);

    CHECK(td_bt_parse_command("Z", &command));
    CHECK(command.action == TD_BT_ACTION_STOP);

    CHECK(td_bt_parse_command("B", &command));
    CHECK(command.action == TD_BT_ACTION_ARC_FORWARD_RIGHT);
    CHECK(td_bt_parse_command("D", &command));
    CHECK(command.action == TD_BT_ACTION_ARC_BACKWARD_RIGHT);
    CHECK(td_bt_parse_command("F", &command));
    CHECK(command.action == TD_BT_ACTION_ARC_BACKWARD_LEFT);
    CHECK(td_bt_parse_command("H", &command));
    CHECK(command.action == TD_BT_ACTION_ARC_FORWARD_LEFT);

    CHECK(td_bt_parse_command("X", &command));
    CHECK(command.action == TD_BT_ACTION_SPEED_UP);

    CHECK(td_bt_parse_command("Y", &command));
    CHECK(command.action == TD_BT_ACTION_SPEED_DOWN);

    CHECK(td_bt_parse_command("I", &command));
    CHECK(td_bt_parse_command("J", &command));
    CHECK(td_bt_parse_command("K", &command));
    CHECK(command.action == TD_BT_ACTION_IGNORE);
}

static void test_control_commands(void)
{
    td_bt_command_t command;

    CHECK(td_bt_parse_command("S", &command));
    CHECK(command.action == TD_BT_ACTION_STOP);
    CHECK(command.left_percent == 0);
    CHECK(command.right_percent == 0);

    CHECK(td_bt_parse_command("STOP", &command));
    CHECK(command.action == TD_BT_ACTION_STOP);

    CHECK(td_bt_parse_command("ESTOP", &command));
    CHECK(command.action == TD_BT_ACTION_ESTOP);

    CHECK(td_bt_parse_command("CLEAR", &command));
    CHECK(command.action == TD_BT_ACTION_CLEAR_ESTOP);

    CHECK(td_bt_parse_command("LINE", &command));
    CHECK(command.action == TD_BT_ACTION_LINE_FOLLOW);

    CHECK(td_bt_parse_command("FOLLOW", &command));
    CHECK(command.action == TD_BT_ACTION_LINE_FOLLOW);

    CHECK(td_bt_parse_command("?", &command));
    CHECK(command.action == TD_BT_ACTION_HELP);

    CHECK(td_bt_parse_command("HELP", &command));
    CHECK(command.action == TD_BT_ACTION_HELP);
}

static void test_rejects_invalid_commands(void)
{
    td_bt_command_t command;

    CHECK(!td_bt_parse_command("", &command));
    CHECK(!td_bt_parse_command("F0", &command));
    CHECK(!td_bt_parse_command("F101", &command));
    CHECK(!td_bt_parse_command("F12x", &command));
    CHECK(!td_bt_parse_command("Q", &command));
    CHECK(!td_bt_parse_command("S20", &command));
}

static void test_formats_wheeltec_app_status(void)
{
    const char *expected = "{A23:35:67:12}$";
    char output[40];
    char small[8] = "keep";

    CHECK(td_bt_format_wheeltec_status(
              23, 35, 67, 12, output, sizeof(output)) == 15u);
    CHECK(strcmp(output, expected) == 0);
    CHECK(td_bt_format_wheeltec_status(
              23, 35, 67, 12, small, sizeof(small)) == 0u);
    CHECK(small[0] == '\0');
}

int main(void)
{
    test_movement_commands();
    test_wheeltec_app_commands();
    test_control_commands();
    test_rejects_invalid_commands();
    test_formats_wheeltec_app_status();
    printf("checks=%d failures=%d\n", s_checks, s_failures);
    return s_failures == 0 ? 0 : 1;
}
