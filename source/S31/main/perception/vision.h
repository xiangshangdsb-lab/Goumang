#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "control/line_follower.h"

typedef struct {
    line_follow_result_t line;
} td_vision_result_t;

esp_err_t vision_start(void);
void vision_set_threshold(uint8_t threshold);
void vision_process_frame(const uint8_t *rgb565,
                          uint32_t width,
                          uint32_t height);
