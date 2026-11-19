#pragma once

#include <stdint.h>

/* CRC-8 polynomial 0x07 with initial value zero. */
static inline uint8_t c30d_crc8_step(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t bit = 0u; bit < 8u; ++bit)
    {
        if ((crc & 0x80u) != 0u)
        {
            crc = (uint8_t)((crc << 1) ^ 0x07u);
        }
        else
        {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint8_t c30d_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < length; ++i)
    {
        crc = c30d_crc8_step(crc, data[i]);
    }
    return crc;
}
