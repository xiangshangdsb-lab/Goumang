#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint16_t roi_top;
    uint16_t roi_bottom;
    uint8_t threshold;
    uint8_t sample_step;
    uint16_t min_dark_pixels;
    bool dark_is_line;
} line_follow_config_t;

typedef struct {
    bool line_found;
    int16_t error;       /* -100 (line left) .. +100 (line right) */
    uint8_t confidence;  /* 0..100, heuristic coverage score */
    uint16_t centroid_x;
    uint32_t dark_pixels;
} line_follow_result_t;

void line_follow_default_config(line_follow_config_t *config);

bool line_follow_compute_gray(const uint8_t *gray,
                              size_t stride,
                              size_t width,
                              size_t height,
                              const line_follow_config_t *config,
                              line_follow_result_t *result);
