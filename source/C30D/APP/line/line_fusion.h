#pragma once

#include <stdbool.h>
#include <stdint.h>

#define LINE_CCD_PIXELS              128u
#define LINE_RECORD_PERIOD_MS        20u
#define LINE_HISTORY_SECONDS         2u
#define LINE_HISTORY_SAMPLES \
    (LINE_HISTORY_SECONDS * 1000u / LINE_RECORD_PERIOD_MS)
#define LINE_OBSTACLE_STOP_CM        5u
#define LINE_OBSTACLE_CLEAR_CM       8u
#define LINE_BRAKE_MS                100u
#define LINE_CLEAR_CONFIRM_MS        1000u
#define LINE_LOST_STOP_MS            450u
#define LINE_MAX_SPEED_MM_S          350

typedef struct
{
    bool found;
    uint8_t center;
    int16_t error;
    uint8_t width;
    uint8_t contrast;
    uint8_t confidence;
} lf_result_t;

typedef enum
{
    LINE_PHASE_IDLE = 0,
    LINE_PHASE_NORMAL,
    LINE_PHASE_BRAKE,
    LINE_PHASE_REWIND,
    LINE_PHASE_WAIT_CLEAR
} line_phase_t;

typedef enum
{
    LINE_SOURCE_NONE = 0,
    LINE_SOURCE_CCD,
    LINE_SOURCE_LF04,
    LINE_SOURCE_FUSED,
    LINE_SOURCE_LOST,
    LINE_SOURCE_REWIND,
    LINE_SOURCE_OBSTACLE
} line_source_t;

typedef enum
{
    LINE_GUARD_STOP = 0,
    LINE_GUARD_ALLOW
} line_guard_t;

typedef struct
{
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
} line_history_entry_t;

typedef struct
{
    bool enabled;
    bool ccd_valid;
    lf_result_t ccd;
    uint8_t lf_bits;
    bool obstacle_valid;
    uint16_t distance_cm;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    uint8_t speed_percent;
    int8_t vision_bias_percent;
    uint16_t elapsed_ms;
} line_motion_input_t;

typedef struct
{
    int8_t left_percent;
    int8_t right_percent;
    int16_t line_error;
    uint16_t phase_ms;
    uint16_t history_count_ms;
    uint8_t source;
    uint8_t phase;
    uint8_t rewind_progress;
    bool line_active;
    bool obstacle_latched;
} line_motion_output_t;

typedef struct
{
    line_history_entry_t history[LINE_HISTORY_SAMPLES];
    uint16_t history_head;
    uint16_t history_count;
    uint16_t record_elapsed_ms;
    uint16_t phase_ms;
    uint16_t replay_remaining;
    uint16_t replay_elapsed_ms;
    uint16_t clear_elapsed_ms;
    uint16_t lost_elapsed_ms;
    int8_t replay_left_percent;
    int8_t replay_right_percent;
    line_phase_t phase;
    bool obstacle_latched;
} line_motion_t;

/* Runtime tuning applied from the v3.10.0 C30D parameter service. */
void line_fusion_set_turn_gain(uint8_t gain);
void line_fusion_set_max_speed_mm_s(uint16_t mm_s);

uint8_t line_fusion_analyze_ccd(const uint16_t pixels[LINE_CCD_PIXELS],
                                lf_result_t *result);
int16_t line_fusion_lf_error(uint8_t line_bits);
line_guard_t line_motion_ultrasonic_guard(bool valid,
                                          uint16_t distance_cm,
                                          int8_t left_percent,
                                          int8_t right_percent);
void line_motion_init(line_motion_t *state);
void line_motion_step(line_motion_t *state,
                      const line_motion_input_t *input,
                      line_motion_output_t *output);

