#include "line_fusion.h"

#include <stdlib.h>
#include <string.h>

static int16_t clamp_i16(int32_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return (int16_t)value;
}

/*
 * Runtime line-follow tuning (v3.10.0 parameter service).
 *
 * The defaults keep the historical host-test behaviour; the C30D firmware
 * applies the persisted parameter record at boot through the setters below.
 */
static uint8_t s_turn_gain = 32u;
static uint16_t s_max_speed_mm_s = LINE_MAX_SPEED_MM_S;

void line_fusion_set_turn_gain(uint8_t gain)
{
    s_turn_gain = gain;
}

void line_fusion_set_max_speed_mm_s(uint16_t mm_s)
{
    s_max_speed_mm_s = (mm_s == 0u) ? 1u : mm_s;
}

static void history_reset(line_motion_t *state)
{
    state->history_head = 0u;
    state->history_count = 0u;
    state->record_elapsed_ms = 0u;
}

static void history_push(line_motion_t *state, const line_motion_input_t *input)
{
    state->history[state->history_head].left_speed_mm_s =
        input->left_speed_mm_s;
    state->history[state->history_head].right_speed_mm_s =
        input->right_speed_mm_s;
    state->history_head =
        (uint16_t)((state->history_head + 1u) % LINE_HISTORY_SAMPLES);
    if (state->history_count < LINE_HISTORY_SAMPLES)
    {
        ++state->history_count;
    }
}

static void history_record(line_motion_t *state,
                           const line_motion_input_t *input)
{
    state->record_elapsed_ms =
        (uint16_t)(state->record_elapsed_ms + input->elapsed_ms);
    while (state->record_elapsed_ms >= LINE_RECORD_PERIOD_MS)
    {
        history_push(state, input);
        state->record_elapsed_ms =
            (uint16_t)(state->record_elapsed_ms - LINE_RECORD_PERIOD_MS);
    }
}

static int16_t history_left_percent(const line_motion_t *state,
                                    uint16_t newest_offset)
{
    uint16_t index;
    int32_t mm_s;

    index = (uint16_t)((state->history_head + LINE_HISTORY_SAMPLES -
                        newest_offset) % LINE_HISTORY_SAMPLES);
    mm_s = state->history[index].left_speed_mm_s;
    return clamp_i16((-mm_s * 100) / (int32_t)s_max_speed_mm_s, -100, 100);
}

static int16_t history_right_percent(const line_motion_t *state,
                                     uint16_t newest_offset)
{
    uint16_t index;
    int32_t mm_s;

    index = (uint16_t)((state->history_head + LINE_HISTORY_SAMPLES -
                        newest_offset) % LINE_HISTORY_SAMPLES);
    mm_s = state->history[index].right_speed_mm_s;
    return clamp_i16((-mm_s * 100) / (int32_t)s_max_speed_mm_s, -100, 100);
}

static uint8_t lf_black_count(uint8_t line_bits)
{
    uint8_t black = (uint8_t)(~line_bits & 0x0Fu);
    uint8_t count = 0u;
    uint8_t i;

    for (i = 0u; i < 4u; ++i)
    {
        if ((black & (uint8_t)(1u << i)) != 0u)
        {
            ++count;
        }
    }
    return count;
}

uint8_t line_fusion_analyze_ccd(const uint16_t pixels[LINE_CCD_PIXELS],
                                lf_result_t *result)
{
    uint16_t minimum;
    uint16_t maximum;
    uint16_t threshold;
    uint16_t i;
    uint16_t run_start;
    uint16_t run_length;
    uint16_t best_start;
    uint16_t best_length;
    uint16_t best_score;
    uint16_t contrast;
    int32_t centroid_sum;
    uint8_t center;
    int32_t error;

    if (result == NULL)
    {
        return 0u;
    }
    memset(result, 0, sizeof(*result));
    result->center = LINE_CCD_PIXELS / 2u;
    if (pixels == NULL)
    {
        return 0u;
    }

    minimum = 65535u;
    maximum = 0u;
    for (i = 4u; i < LINE_CCD_PIXELS - 4u; ++i)
    {
        if (pixels[i] < minimum)
        {
            minimum = pixels[i];
        }
        if (pixels[i] > maximum)
        {
            maximum = pixels[i];
        }
    }

    contrast = (uint16_t)(maximum - minimum);
    if (contrast > 255u)
    {
        contrast = 255u;
    }
    result->contrast = (uint8_t)contrast;
    if (contrast < 12u)
    {
        return 1u;
    }

    threshold = (uint16_t)(minimum + contrast * 55u / 100u);
    best_start = 0u;
    best_length = 0u;
    best_score = 0u;
    run_start = 4u;
    run_length = 0u;

    for (i = 4u; i <= LINE_CCD_PIXELS - 4u; ++i)
    {
        bool dark = i < LINE_CCD_PIXELS - 4u && pixels[i] <= threshold;

        if (dark)
        {
            if (run_length == 0u)
            {
                run_start = i;
            }
            ++run_length;
        }
        else if (run_length > 0u)
        {
            uint16_t run_center =
                (uint16_t)(run_start + (run_length - 1u) / 2u);
            uint16_t distance =
                run_center > LINE_CCD_PIXELS / 2u ?
                (uint16_t)(run_center - LINE_CCD_PIXELS / 2u) :
                (uint16_t)(LINE_CCD_PIXELS / 2u - run_center);
            uint16_t score = (uint16_t)(run_length * 8u);

            if (score >= distance / 4u)
            {
                score = (uint16_t)(score - distance / 4u);
            }
            else
            {
                score = 0u;
            }
            if (run_length >= 2u && score > best_score)
            {
                best_start = run_start;
                best_length = run_length;
                best_score = score;
            }
            run_length = 0u;
        }
    }

    if (best_length < 2u)
    {
        return 1u;
    }

    centroid_sum = 0;
    for (i = best_start; i < best_start + best_length; ++i)
    {
        centroid_sum += i;
    }
    center = (uint8_t)(centroid_sum / best_length);
    result->found = true;
    result->center = center;
    result->width = (uint8_t)(best_length > 255u ? 255u : best_length);
    error = ((int32_t)center - (int32_t)(LINE_CCD_PIXELS / 2u)) * 100;
    error /= (int32_t)(LINE_CCD_PIXELS / 2u - 1u);
    result->error = clamp_i16(error, -100, 100);
    result->confidence =
        (uint8_t)(contrast * 2u + best_length * 4u > 100u ?
                  100u : contrast * 2u + best_length * 4u);
    return 1u;
}

int16_t line_fusion_lf_error(uint8_t line_bits)
{
    static const int16_t weights[4] = {-100, -33, 33, 100};
    uint8_t black = (uint8_t)(~line_bits & 0x0Fu);
    int16_t weighted = 0;
    uint8_t count = 0u;
    uint8_t i;

    for (i = 0u; i < 4u; ++i)
    {
        if ((black & (uint8_t)(1u << (3u - i))) != 0u)
        {
            weighted = (int16_t)(weighted + weights[i]);
            ++count;
        }
    }
    return count == 0u ? 0 : (int16_t)(weighted / count);
}

line_guard_t line_motion_ultrasonic_guard(bool valid,
                                          uint16_t distance_cm,
                                          int8_t left_percent,
                                          int8_t right_percent)
{
    if (!valid || distance_cm == 0u)
    {
        /* Ultrasonic is optional; unknown distance is not an obstacle. */
        return LINE_GUARD_ALLOW;
    }
    if (distance_cm <= LINE_OBSTACLE_STOP_CM &&
        (left_percent > 0 || right_percent > 0))
    {
        return LINE_GUARD_STOP;
    }
    return LINE_GUARD_ALLOW;
}

void line_motion_init(line_motion_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

static void reset_idle(line_motion_t *state, line_motion_output_t *output)
{
    state->phase = LINE_PHASE_IDLE;
    state->phase_ms = 0u;
    state->replay_remaining = 0u;
    state->replay_elapsed_ms = 0u;
    state->clear_elapsed_ms = 0u;
    state->lost_elapsed_ms = 0u;
    state->replay_left_percent = 0;
    state->replay_right_percent = 0;
    state->obstacle_latched = false;
    history_reset(state);
    output->left_percent = 0;
    output->right_percent = 0;
    output->line_active = false;
    output->obstacle_latched = false;
}

static void run_normal(line_motion_t *state,
                       const line_motion_input_t *input,
                       line_motion_output_t *output)
{
    int16_t lf_error = line_fusion_lf_error(input->lf_bits);
    bool lf_present = lf_black_count(input->lf_bits) != 0u;
    bool ccd_present = input->ccd_valid && input->ccd.found;
    int16_t error = 0;
    int16_t base;
    int16_t turn;
    int16_t left;
    int16_t right;
    uint8_t speed = input->speed_percent;

    if (speed < 5u)
    {
        speed = 5u;
    }
    else if (speed > 100u)
    {
        speed = 100u;
    }

    if (ccd_present && lf_present)
    {
        error = (int16_t)((input->ccd.error * 4 + lf_error) / 5);
        output->source = LINE_SOURCE_FUSED;
    }
    else if (ccd_present)
    {
        error = input->ccd.error;
        output->source = LINE_SOURCE_CCD;
    }
    else if (lf_present)
    {
        error = lf_error;
        output->source = LINE_SOURCE_LF04;
    }
    else
    {
        state->lost_elapsed_ms =
            (uint16_t)(state->lost_elapsed_ms + input->elapsed_ms);
        output->source = LINE_SOURCE_LOST;
        output->line_active = false;
        output->left_percent = 0;
        output->right_percent = 0;
        if (state->lost_elapsed_ms >= LINE_LOST_STOP_MS)
        {
            state->lost_elapsed_ms = LINE_LOST_STOP_MS;
        }
        output->phase_ms = state->lost_elapsed_ms;
        return;
    }

    state->lost_elapsed_ms = 0u;
    output->line_error = error;
    output->line_active = true;
    base = (int16_t)(speed * (100 - abs(error) / 4) / 100);
    if (base < 12)
    {
        base = 12;
    }
    turn = (int16_t)(-(error * (int16_t)s_turn_gain) / 100);
    left = clamp_i16(base - turn, -100, 100);
    right = clamp_i16(base + turn, -100, 100);
    /* CCD 有效时才加有限的前馈修正。保留原均速和停车路径。 */
    int16_t bias = input->vision_bias_percent;
    if (bias >= -5 && bias <= 5 && speed <= 15u && ccd_present &&
        input->obstacle_valid && input->distance_cm > 25u &&
        left + bias >= 0 && right - bias >= 0 &&
        left + bias <= 20 && right - bias <= 20)
    {
        left += bias;
        right -= bias;
    }
    output->left_percent = (int8_t)left;
    output->right_percent = (int8_t)right;
}

static void start_brake(line_motion_t *state, line_motion_output_t *output)
{
    state->phase = LINE_PHASE_BRAKE;
    state->phase_ms = 0u;
    state->obstacle_latched = true;
    state->replay_remaining = state->history_count;
    state->replay_elapsed_ms = 0u;
    state->clear_elapsed_ms = 0u;
    state->replay_left_percent = 0;
    state->replay_right_percent = 0;
    output->source = LINE_SOURCE_OBSTACLE;
    output->obstacle_latched = true;
    output->left_percent = 0;
    output->right_percent = 0;
    output->line_active = false;
}

void line_motion_step(line_motion_t *state,
                      const line_motion_input_t *input,
                      line_motion_output_t *output)
{
    uint16_t step_ms;

    if (output == NULL)
    {
        return;
    }
    memset(output, 0, sizeof(*output));
    if (state == NULL || input == NULL)
    {
        return;
    }
    if (!input->enabled)
    {
        reset_idle(state, output);
        return;
    }

    step_ms = input->elapsed_ms == 0u ? 10u : input->elapsed_ms;

    if (state->phase == LINE_PHASE_IDLE)
    {
        state->phase = LINE_PHASE_NORMAL;
    }

    if (state->phase == LINE_PHASE_NORMAL &&
        input->obstacle_valid &&
        input->distance_cm <= LINE_OBSTACLE_STOP_CM)
    {
        start_brake(state, output);
        output->phase = state->phase;
        return;
    }

    if (state->phase == LINE_PHASE_NORMAL)
    {
        history_record(state, input);
        run_normal(state, input, output);
    }
    else if (state->phase == LINE_PHASE_BRAKE)
    {
        state->phase_ms = (uint16_t)(state->phase_ms + step_ms);
        output->left_percent = 0;
        output->right_percent = 0;
        output->source = LINE_SOURCE_OBSTACLE;
        output->obstacle_latched = true;
        if (state->phase_ms >= LINE_BRAKE_MS)
        {
            state->phase = LINE_PHASE_REWIND;
            state->phase_ms = 0u;
            state->replay_elapsed_ms = 0u;
        }
    }
    else if (state->phase == LINE_PHASE_REWIND)
    {
        state->replay_elapsed_ms =
            (uint16_t)(state->replay_elapsed_ms + step_ms);
        output->left_percent = 0;
        output->right_percent = 0;
        output->source = LINE_SOURCE_REWIND;
        output->obstacle_latched = true;
        while (state->replay_elapsed_ms >= LINE_RECORD_PERIOD_MS &&
               state->replay_remaining > 0u)
        {
            uint16_t newest_offset =
                (uint16_t)(state->history_count - state->replay_remaining + 1u);
            state->replay_left_percent =
                (int8_t)history_left_percent(state, newest_offset);
            state->replay_right_percent =
                (int8_t)history_right_percent(state, newest_offset);
            output->left_percent = state->replay_left_percent;
            output->right_percent = state->replay_right_percent;
            state->replay_elapsed_ms =
                (uint16_t)(state->replay_elapsed_ms - LINE_RECORD_PERIOD_MS);
            --state->replay_remaining;
            ++state->phase_ms;
        }
        output->rewind_progress =
            state->history_count == 0u ? 100u :
            (uint8_t)(state->phase_ms * 100u / state->history_count);
        if (output->rewind_progress > 100u)
        {
            output->rewind_progress = 100u;
        }
        if (state->replay_remaining > 0u ||
            (state->history_count > 0u &&
             state->replay_elapsed_ms < LINE_RECORD_PERIOD_MS))
        {
            output->left_percent = state->replay_left_percent;
            output->right_percent = state->replay_right_percent;
        }
        else if (state->replay_elapsed_ms >= LINE_RECORD_PERIOD_MS)
        {
            state->phase = LINE_PHASE_WAIT_CLEAR;
            state->phase_ms = 0u;
            state->clear_elapsed_ms = 0u;
            history_reset(state);
            state->replay_left_percent = 0;
            state->replay_right_percent = 0;
            output->left_percent = 0;
            output->right_percent = 0;
        }
    }
    else if (state->phase == LINE_PHASE_WAIT_CLEAR)
    {
        output->left_percent = 0;
        output->right_percent = 0;
        output->source = LINE_SOURCE_OBSTACLE;
        output->obstacle_latched = true;
        if (input->obstacle_valid &&
            input->distance_cm > LINE_OBSTACLE_CLEAR_CM)
        {
            state->clear_elapsed_ms =
                (uint16_t)(state->clear_elapsed_ms + step_ms);
        }
        else
        {
            state->clear_elapsed_ms = 0u;
        }
        if (state->clear_elapsed_ms >= LINE_CLEAR_CONFIRM_MS)
        {
            state->phase = LINE_PHASE_NORMAL;
            state->obstacle_latched = false;
            state->clear_elapsed_ms = 0u;
            state->lost_elapsed_ms = 0u;
            output->obstacle_latched = false;
            output->left_percent = 0;
            output->right_percent = 0;
            output->source = LINE_SOURCE_NONE;
        }
    }

    output->phase = state->phase;
}

