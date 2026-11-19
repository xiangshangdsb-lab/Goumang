#pragma once

#include <stddef.h>
#include <stdint.h>
#include "../common/crc8.h"

/*
 * v3.10.0 C30D flash record: gimbal step position plus the five runtime
 * parameters.  The layout is also exercised by tests/test_pan_param.c through
 * the pure helpers below, so firmware and host test share one definition.
 *
 * The record is stored at STMFLASH_NVM_ADDR (see stmflash.h).  A CRC8 over
 * every byte that precedes the crc field rejects torn or stale pages.
 */

#define C30D_NVM_MAGIC       0x44333043u   /* "C30D" */
#define C30D_NVM_LEGACY      0x0310u
#define C30D_NVM_VERSION     0x0311u
#define C30D_NVM_PAN_MOVING  1u
#define C30D_PARAM_COUNT     5u
#define C30D_PARAM_FRAME_LEN 5u

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int32_t pan_steps;
    uint16_t line_base_percent;
    uint16_t line_turn_gain;
    uint16_t speed_kp;
    uint16_t speed_ki;
    uint16_t max_speed_mms;
    uint16_t reserved;
    uint32_t crc;
} C30D_NvmRecord;

/* Number of bytes protected by the CRC, i.e. the offset of the crc field. */
static inline uint8_t c30d_nvm_crc_length(void)
{
    return (uint8_t)offsetof(C30D_NvmRecord, crc);
}

static inline uint8_t c30d_nvm_crc(const C30D_NvmRecord *record)
{
    return c30d_crc8((const uint8_t *)record, c30d_nvm_crc_length());
}

static inline void c30d_nvm_set_crc(C30D_NvmRecord *record)
{
    record->crc = (uint32_t)c30d_nvm_crc(record);
}

static inline uint8_t c30d_nvm_valid(const C30D_NvmRecord *record)
{
    if (record->magic != C30D_NVM_MAGIC ||
        (record->version != C30D_NVM_VERSION &&
         record->version != C30D_NVM_LEGACY) ||
        record->reserved > (record->version == C30D_NVM_LEGACY ? 0u : C30D_NVM_PAN_MOVING) ||
        record->size != (uint16_t)sizeof(C30D_NvmRecord))
    {
        return 0u;
    }
    return (record->crc == (uint32_t)c30d_nvm_crc(record)) ? 1u : 0u;
}

/* Only a completed, in-range move is a usable origin for power-up return. */
static inline uint8_t c30d_nvm_pan_known(const C30D_NvmRecord *record,
                                         int32_t steps_per_180_deg)
{
    if (!c30d_nvm_valid(record) || record->reserved != 0u)
    {
        return 0u;
    }
    int32_t limit = record->version == C30D_NVM_LEGACY ?
                    steps_per_180_deg / 2 : steps_per_180_deg * 3 / 2;
    return record->pan_steps >= -limit && record->pan_steps <= limit;
}
