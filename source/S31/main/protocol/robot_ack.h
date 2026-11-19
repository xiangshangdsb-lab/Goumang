#pragma once

#include "robot_protocol.h"

/* One ordinary command is in flight. The transport supplies synchronization. */
typedef struct {
    uint8_t command;
    uint8_t status;
    bool received;
} rp_ack_latch_t;

static inline bool rp_ack_offer(rp_ack_latch_t *latch, const rp_command_ack_t *ack)
{
    if (latch->command == 0u || latch->command != ack->command || latch->received) {
        return false;
    }
    latch->status = ack->status;
    latch->received = true;
    return true;
}
