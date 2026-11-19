#include "bluetooth_control.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static char command_letter(char value)
{
    return (char)toupper((unsigned char)value);
}

static bool parse_speed(const char *text, int *speed)
{
    int value = 0;
    uint8_t digits = 0u;

    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    while (*text >= '0' && *text <= '9') {
        value = value * 10 + (*text - '0');
        ++text;
        ++digits;
        if (digits > 3u) {
            return false;
        }
    }
    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    if (*text != '\0' || value < 1 || value > 100) {
        return false;
    }
    *speed = value;
    return true;
}

static bool has_only_spaces(const char *text)
{
    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    return *text == '\0';
}

static bool equals_ignore_case(const char *line, const char *word)
{
    while (*line != '\0' && *word != '\0') {
        if (command_letter(*line) != command_letter(*word)) {
            return false;
        }
        ++line;
        ++word;
    }
    while (*line == ' ' || *line == '\t') {
        ++line;
    }
    return *line == '\0' && *word == '\0';
}

bool td_bt_parse_command(const char *line, td_bt_command_t *command)
{
    const char *argument;
    char letter;
    int speed = TD_BT_DEFAULT_SPEED_PERCENT;

    if (line == NULL || command == NULL) {
        return false;
    }
    memset(command, 0, sizeof(*command));

    while (*line == ' ' || *line == '\t') {
        ++line;
    }
    if (*line == '\0') {
        return false;
    }

    if (equals_ignore_case(line, "ESTOP")) {
        command->action = TD_BT_ACTION_ESTOP;
        return true;
    }
    if (equals_ignore_case(line, "STOP")) {
        command->action = TD_BT_ACTION_STOP;
        return true;
    }
    if (equals_ignore_case(line, "CLEAR")) {
        command->action = TD_BT_ACTION_CLEAR_ESTOP;
        return true;
    }
    if (equals_ignore_case(line, "LINE") ||
        equals_ignore_case(line, "FOLLOW")) {
        command->action = TD_BT_ACTION_LINE_FOLLOW;
        return true;
    }
    if (equals_ignore_case(line, "HELP")) {
        command->action = TD_BT_ACTION_HELP;
        return true;
    }

    letter = command_letter(*line++);
    argument = line;
    if (letter == 'F' || letter == 'B' || letter == 'L' || letter == 'R') {
        if (!has_only_spaces(argument)) {
            if (!parse_speed(argument, &speed)) {
                return false;
            }
            if (letter == 'F') {
                command->action = TD_BT_ACTION_FORWARD;
                command->left_percent = (int8_t)speed;
                command->right_percent = (int8_t)speed;
            } else if (letter == 'B') {
                command->action = TD_BT_ACTION_BACKWARD;
                command->left_percent = (int8_t)-speed;
                command->right_percent = (int8_t)-speed;
            } else if (letter == 'L') {
                command->action = TD_BT_ACTION_TURN_LEFT;
                command->left_percent = (int8_t)-speed;
                command->right_percent = (int8_t)speed;
            } else {
                command->action = TD_BT_ACTION_TURN_RIGHT;
                command->left_percent = (int8_t)speed;
                command->right_percent = (int8_t)-speed;
            }
            command->explicit_speed = true;
            return true;
        }
    }

    if (!has_only_spaces(argument)) {
        return false;
    }
    if (letter == 'A') {
        command->action = TD_BT_ACTION_FORWARD;
        command->left_percent = TD_BT_DEFAULT_SPEED_PERCENT;
        command->right_percent = TD_BT_DEFAULT_SPEED_PERCENT;
    } else if (letter == 'E') {
        command->action = TD_BT_ACTION_BACKWARD;
        command->left_percent = -TD_BT_DEFAULT_SPEED_PERCENT;
        command->right_percent = -TD_BT_DEFAULT_SPEED_PERCENT;
    } else if (letter == 'C') {
        command->action = TD_BT_ACTION_TURN_RIGHT;
        command->left_percent = TD_BT_DEFAULT_SPEED_PERCENT;
        command->right_percent = -TD_BT_DEFAULT_SPEED_PERCENT;
    } else if (letter == 'G') {
        command->action = TD_BT_ACTION_TURN_LEFT;
        command->left_percent = -TD_BT_DEFAULT_SPEED_PERCENT;
        command->right_percent = TD_BT_DEFAULT_SPEED_PERCENT;
    } else if (letter == 'B') {
        command->action = TD_BT_ACTION_ARC_FORWARD_RIGHT;
        command->left_percent = 20;
        command->right_percent = 35;
    } else if (letter == 'D') {
        command->action = TD_BT_ACTION_ARC_BACKWARD_RIGHT;
        command->left_percent = -20;
        command->right_percent = -35;
    } else if (letter == 'F') {
        command->action = TD_BT_ACTION_ARC_BACKWARD_LEFT;
        command->left_percent = -35;
        command->right_percent = -20;
    } else if (letter == 'H') {
        command->action = TD_BT_ACTION_ARC_FORWARD_LEFT;
        command->left_percent = 35;
        command->right_percent = 20;
    } else if (letter == 'X') {
        command->action = TD_BT_ACTION_SPEED_UP;
    } else if (letter == 'Y') {
        command->action = TD_BT_ACTION_SPEED_DOWN;
    } else if (letter == 'I' || letter == 'J' || letter == 'K') {
        command->action = TD_BT_ACTION_IGNORE;
    } else if (letter == 'S' || letter == 'Z') {
        command->action = TD_BT_ACTION_STOP;
    } else if (letter == '?' || letter == 'H') {
        command->action = TD_BT_ACTION_HELP;
    } else {
        return false;
    }
    return true;
}

size_t td_bt_format_wheeltec_status(int left_speed, int right_speed,
                                    int battery_percent, int angle_deg,
                                    char *out, size_t capacity)
{
    int written;

    if (out == NULL || capacity == 0u) {
        return 0u;
    }
    written = snprintf(out, capacity, "{A%d:%d:%d:%d}$",
                       left_speed, right_speed,
                       battery_percent, angle_deg);
    if (written < 0 || (size_t)written >= capacity) {
        out[0] = '\0';
        return 0u;
    }
    return (size_t)written;
}
