#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TD_LINE_IMAGE_ROWS 3
#define TD_LINE_IMAGE_COLS 8
#define TD_LINE_IMAGE_FEATURES (TD_LINE_IMAGE_ROWS * TD_LINE_IMAGE_COLS)
#define TD_LINE_MODEL_INPUTS (TD_LINE_IMAGE_FEATURES + 2)
#define TD_LINE_MODEL_HIDDEN1 16
#define TD_LINE_MODEL_HIDDEN2 8

typedef struct {
    int64_t frame_us;
    int64_t ccd_us;
    uint8_t dark[TD_LINE_IMAGE_FEATURES];
    uint8_t ccd_center;
    uint8_t ccd_confidence;
    int16_t speed_mm_s;
    int8_t prediction;
    bool valid;
} td_line_sample_t;

#include "line_learning_weights.h"

#if TD_LINE_MODEL_AVAILABLE
static inline void td_line_dense(const float *inputs, unsigned input_count,
                                 const float *weights, const float *bias,
                                 float *outputs, unsigned output_count)
{
    for (unsigned j = 0; j < output_count; ++j) {
        float value = bias[j];
        for (unsigned i = 0; i < input_count; ++i) {
            value += weights[j * input_count + i] * inputs[i];
        }
        outputs[j] = value > 0.0f ? value : 0.0f;
    }
}

/* The trained output is the predicted CCD offset, in percent of half width. */
static inline int8_t td_line_predict(const td_line_sample_t *sample)
{
    float input[TD_LINE_MODEL_INPUTS];
    for (unsigned i = 0; i < TD_LINE_IMAGE_FEATURES; ++i) {
        input[i] = sample->dark[i] / 100.0f;
    }
    input[TD_LINE_IMAGE_FEATURES] = ((int)sample->ccd_center - 64) / 64.0f;
    input[TD_LINE_IMAGE_FEATURES + 1] = sample->speed_mm_s / 500.0f;
    float h1[TD_LINE_MODEL_HIDDEN1];
    float h2[TD_LINE_MODEL_HIDDEN2];
    td_line_dense(input, TD_LINE_MODEL_INPUTS, td_line_w1, td_line_b1,
                  h1, TD_LINE_MODEL_HIDDEN1);
    td_line_dense(h1, TD_LINE_MODEL_HIDDEN1, td_line_w2, td_line_b2,
                  h2, TD_LINE_MODEL_HIDDEN2);
    float value = td_line_b3;
    for (unsigned i = 0; i < TD_LINE_MODEL_HIDDEN2; ++i) {
        value += td_line_w3[i] * h2[i];
    }
    if (value > 1.0f) {
        value = 1.0f;
    } else if (value < -1.0f) {
        value = -1.0f;
    }
    return (int8_t)(value * 100.0f + (value >= 0.0f ? 0.5f : -0.5f));
}
#endif
