#pragma once

#include <stdint.h>

/*
 * PPA always writes RGB565 in little-endian byte order. Keep every RGB565
 * consumer on this helper so vision, AI and snapshots cannot disagree about
 * the source byte order.
 */
static inline uint16_t td_rgb565_read_le(const uint8_t *pixel)
{
    return (uint16_t)((uint16_t)pixel[0] | ((uint16_t)pixel[1] << 8));
}

static inline void td_rgb565_to_rgb8(uint16_t pixel,
                                     uint8_t *red,
                                     uint8_t *green,
                                     uint8_t *blue)
{
    *red = (uint8_t)((((pixel >> 11) & 0x1Fu) * 255u + 15u) / 31u);
    *green = (uint8_t)((((pixel >> 5) & 0x3Fu) * 255u + 31u) / 63u);
    *blue = (uint8_t)(((pixel & 0x1Fu) * 255u + 15u) / 31u);
}
