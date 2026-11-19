#pragma once

#include <stdint.h>

/*
 * v3.10.0 runtime parameter contract shared by the C30D firmware and the
 * ESP32-S31 host (S31 robot_protocol.h uses the same ids and ranges).
 *
 * The helpers are pure logic so the host test suite (tests/test_pan_param.c)
 * can validate them with gcc; the firmware includes this header from
 * s31_link.c.
 */

enum
{
    PAN_PARAM_LINE_BASE_PERCENT = 1,
    PAN_PARAM_LINE_TURN_GAIN = 2,
    PAN_PARAM_SPEED_KP = 3,
    PAN_PARAM_SPEED_KI = 4,
    PAN_PARAM_MAX_SPEED_MMS = 5,
    PAN_PARAM_COUNT = 5
};

static inline int32_t pan_param_min(uint8_t id)
{
    switch (id)
    {
        case PAN_PARAM_LINE_BASE_PERCENT:
            return 5;
        case PAN_PARAM_LINE_TURN_GAIN:
            return 0;
        case PAN_PARAM_SPEED_KP:
            return 0;
        case PAN_PARAM_SPEED_KI:
            return 0;
        case PAN_PARAM_MAX_SPEED_MMS:
            return 50;
        default:
            return 0;
    }
}

static inline int32_t pan_param_max(uint8_t id)
{
    switch (id)
    {
        case PAN_PARAM_LINE_BASE_PERCENT:
            return 40;
        case PAN_PARAM_LINE_TURN_GAIN:
            return 60;
        case PAN_PARAM_SPEED_KP:
            return 5000;
        case PAN_PARAM_SPEED_KI:
            return 5000;
        case PAN_PARAM_MAX_SPEED_MMS:
            return 350;
        default:
            return 0;
    }
}

static inline int32_t pan_param_default(uint8_t id)
{
    switch (id)
    {
        case PAN_PARAM_LINE_BASE_PERCENT:
            return 30;
        case PAN_PARAM_LINE_TURN_GAIN:
            return 12;
        case PAN_PARAM_SPEED_KP:
            return 700;
        case PAN_PARAM_SPEED_KI:
            return 700;
        case PAN_PARAM_MAX_SPEED_MMS:
            return 350;
        default:
            return 0;
    }
}

static inline uint8_t pan_param_valid(uint8_t id, int32_t value)
{
    return (id >= 1u && id <= (uint8_t)PAN_PARAM_COUNT &&
            value >= pan_param_min(id) && value <= pan_param_max(id)) ? 1u : 0u;
}

enum { PAN_HOME_DEG = 270, PAN_MAX_DEG = 540 };

static inline int32_t pan_param_steps_from_deg(uint16_t pan_deg,
                                               int32_t steps_per_180_deg)
{
    int32_t delta_deg = (int32_t)pan_deg - PAN_HOME_DEG;
    return (delta_deg * steps_per_180_deg +
            (delta_deg >= 0 ? 90 : -90)) / 180;
}

static inline uint16_t pan_param_deg_from_steps(int32_t steps,
                                                int32_t steps_per_180_deg)
{
    int32_t limit = steps_per_180_deg * 3 / 2;
    if (steps <= -limit)
        return 0;
    if (steps >= limit)
        return PAN_MAX_DEG;
    return (uint16_t)(((steps + limit) * 180 + steps_per_180_deg / 2) /
                      steps_per_180_deg);
}
