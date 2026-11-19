#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TD_BT_DEFAULT_SPEED_PERCENT 35
#define TD_BT_COMMAND_TIMEOUT_MS    600u

typedef enum {
    TD_BT_ACTION_NONE = 0,
    TD_BT_ACTION_FORWARD,
    TD_BT_ACTION_BACKWARD,
    TD_BT_ACTION_TURN_LEFT,
    TD_BT_ACTION_TURN_RIGHT,
    TD_BT_ACTION_ARC_FORWARD_LEFT,
    TD_BT_ACTION_ARC_FORWARD_RIGHT,
    TD_BT_ACTION_ARC_BACKWARD_LEFT,
    TD_BT_ACTION_ARC_BACKWARD_RIGHT,
    TD_BT_ACTION_SPEED_UP,
    TD_BT_ACTION_SPEED_DOWN,
    TD_BT_ACTION_STOP,
    TD_BT_ACTION_ESTOP,
    TD_BT_ACTION_CLEAR_ESTOP,
    TD_BT_ACTION_LINE_FOLLOW,
    TD_BT_ACTION_HELP,
    TD_BT_ACTION_IGNORE
} td_bt_action_t;

typedef struct {
    td_bt_action_t action;
    int8_t left_percent;
    int8_t right_percent;
    bool explicit_speed;
} td_bt_command_t;

/*
 * Text commands accepted from a generic Bluetooth serial terminal:
 * WHEELTEC APP single-byte controls use the official mapping:
 *   A forward, E backward, C right, G left, Z stop
 *   B/D/F/H diagonal arcs, X/Y speed up/down, I/J/K page select
 * Numeric console extensions:
 *   F42 forward, B42 backward, L30 left, R30 right
 *   S stop, ESTOP emergency stop, CLEAR clear, ? help
 *   LINE / FOLLOW enables the LF04 four-channel line follower
 */
bool td_bt_parse_command(const char *line, td_bt_command_t *command);
size_t td_bt_format_wheeltec_status(int left_speed, int right_speed,
                                    int battery_percent, int angle_deg,
                                    char *out, size_t capacity);
