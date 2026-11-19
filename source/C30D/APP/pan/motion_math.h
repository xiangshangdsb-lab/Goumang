#pragma once

#include <stdint.h>

static inline uint8_t motion_math_pan_deg_from_steps(
    int32_t steps, int32_t half_range_steps)
{
    int64_t scaled;
    int64_t delta;
    int32_t angle;

    if (half_range_steps <= 0) {
        return 90u;
    }
    scaled = (int64_t)steps * 90;
    if (scaled >= 0) {
        delta = (scaled + half_range_steps / 2) / half_range_steps;
    } else {
        delta = (scaled - half_range_steps / 2) / half_range_steps;
    }
    angle = 90 + (int32_t)delta;
    if (angle < 0) {
        angle = 0;
    } else if (angle > 180) {
        angle = 180;
    }
    return (uint8_t)angle;
}
