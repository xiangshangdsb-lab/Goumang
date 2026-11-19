#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../APP/line/line_fusion.h"
#include "../APP/pan/motion_math.h"

static int s_checks;
static int s_failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        ++s_checks;                                                        \
        if (!(condition)) {                                                \
            ++s_failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
        }                                                                  \
    } while (0)

static void fill_frame(uint16_t pixels[LINE_CCD_PIXELS], int dark_start,
                       int dark_width)
{
    unsigned int i;
    int index;

    for (i = 0; i < LINE_CCD_PIXELS; ++i) {
        pixels[i] = 220u;
    }
    for (index = dark_start; index < dark_start + dark_width; ++index) {
        if (index >= 0 && index < (int)LINE_CCD_PIXELS) {
            pixels[index] = 35u;
        }
    }
}

static void test_ccd_center_and_right(void)
{
    uint16_t pixels[LINE_CCD_PIXELS];
    lf_result_t result;

    fill_frame(pixels, 61, 7);
    CHECK(line_fusion_analyze_ccd(pixels, &result));
    CHECK(result.found);
    CHECK(result.center >= 63u && result.center <= 65u);
    CHECK(result.error >= -2 && result.error <= 2);
    CHECK(result.width >= 7u);
    CHECK(result.confidence >= 20u);

    fill_frame(pixels, 93, 7);
    CHECK(line_fusion_analyze_ccd(pixels, &result));
    CHECK(result.found);
    CHECK(result.center >= 95u && result.center <= 97u);
    CHECK(result.error >= 48 && result.error <= 53);
}

static void test_ccd_rejects_flat_frame(void)
{
    uint16_t pixels[LINE_CCD_PIXELS];
    lf_result_t result;
    unsigned int i;

    for (i = 0; i < LINE_CCD_PIXELS; ++i) {
        pixels[i] = 127u;
    }
    CHECK(line_fusion_analyze_ccd(pixels, &result));
    CHECK(!result.found);
    CHECK(result.center == 64u);
    CHECK(result.error == 0);
    CHECK(!line_fusion_analyze_ccd(NULL, &result));
    CHECK(!line_fusion_analyze_ccd(pixels, NULL));
}

static void test_lf04_error(void)
{
    CHECK(line_fusion_lf_error(0x0Fu) == 0);
    CHECK(line_fusion_lf_error(0x07u) < -80);
    CHECK(line_fusion_lf_error(0x0Bu) < 0);
    CHECK(line_fusion_lf_error(0x0Du) > 0);
    CHECK(line_fusion_lf_error(0x0Eu) > 80);
}

static void test_optional_ultrasonic_guard(void)
{
    CHECK(line_motion_ultrasonic_guard(false, 0u, 30, 30) ==
          LINE_GUARD_ALLOW);
    CHECK(line_motion_ultrasonic_guard(true, 0u, -30, -30) ==
          LINE_GUARD_ALLOW);
    CHECK(line_motion_ultrasonic_guard(true, 5u, 30, 30) ==
          LINE_GUARD_STOP);
    CHECK(line_motion_ultrasonic_guard(true, 5u, -30, -30) ==
          LINE_GUARD_ALLOW);
    CHECK(line_motion_ultrasonic_guard(true, 6u, 30, 30) ==
          LINE_GUARD_ALLOW);
}

static void test_pan_angle_uses_actual_step_position(void)
{
    CHECK(motion_math_pan_deg_from_steps(0, 800) == 90u);
    CHECK(motion_math_pan_deg_from_steps(800, 800) == 180u);
    CHECK(motion_math_pan_deg_from_steps(-800, 800) == 0u);
    CHECK(motion_math_pan_deg_from_steps(900, 800) == 180u);
    CHECK(motion_math_pan_deg_from_steps(-900, 800) == 0u);
}

static line_motion_input_t normal_input(void)
{
    line_motion_input_t input;

    memset(&input, 0, sizeof(input));
    input.enabled = true;
    input.ccd_valid = true;
    input.ccd.found = true;
    input.ccd.center = 64u;
    input.ccd.error = 0;
    input.ccd.confidence = 80u;
    input.lf_bits = 0x0Fu;
    input.speed_percent = 30u;
    input.elapsed_ms = 20u;
    return input;
}

static void test_ccd_is_primary_and_lf04_supplements(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;

    line_motion_init(&state);
    input.ccd.error = 60;
    input.lf_bits = 0x07u;
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_NORMAL);
    CHECK(output.line_active);
    CHECK(output.source == LINE_SOURCE_FUSED);
    CHECK(output.line_error > 20 && output.line_error < 60);

    input.ccd_valid = false;
    input.lf_bits = 0x0Eu;
    line_motion_step(&state, &input, &output);
    CHECK(output.source == LINE_SOURCE_LF04);
    CHECK(output.line_error > 0);

    input.ccd_valid = true;
    input.ccd.found = false;
    input.lf_bits = 0x0Fu;
    line_motion_step(&state, &input, &output);
    CHECK(output.source == LINE_SOURCE_LOST);
    CHECK(output.left_percent == 0);
    CHECK(output.right_percent == 0);
}

static void test_lf04_center_two_sensors_are_still_a_line(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;

    line_motion_init(&state);
    input.ccd_valid = false;
    input.ccd.found = false;
    input.lf_bits = 0x09u; /* Center two sensors see black. */
    line_motion_step(&state, &input, &output);
    CHECK(line_fusion_lf_error(input.lf_bits) == 0);
    CHECK(output.line_active);
    CHECK(output.source == LINE_SOURCE_LF04);
    CHECK(output.left_percent > 0);
    CHECK(output.right_percent > 0);
}

static void populate_history(line_motion_t *state)
{
    line_motion_input_t input = normal_input();
    line_motion_output_t output;
    unsigned int i;

    for (i = 0; i < LINE_HISTORY_SAMPLES; ++i) {
        input.left_speed_mm_s = (int16_t)(100 + i);
        input.right_speed_mm_s = (int16_t)(40 + i);
        line_motion_step(state, &input, &output);
    }
}

static void test_obstacle_brakes_then_replays_newest_first(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;

    line_motion_init(&state);
    populate_history(&state);
    CHECK(state.history_count == LINE_HISTORY_SAMPLES);

    input.obstacle_valid = true;
    input.distance_cm = 5u;
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_BRAKE);
    CHECK(output.left_percent == 0);
    CHECK(output.right_percent == 0);
    CHECK(output.obstacle_latched);

    input.elapsed_ms = 100u;
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_REWIND);
    CHECK(output.left_percent == 0);
    CHECK(output.right_percent == 0);

    input.elapsed_ms = 20u;
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_REWIND);
    CHECK(output.left_percent < 0);
    CHECK(output.right_percent < 0);
    CHECK(output.left_percent <= -56 && output.left_percent >= -60);
    CHECK(output.right_percent <= -39 && output.right_percent >= -43);
    CHECK(output.rewind_progress == 1u);
}

static void test_obstacle_replays_every_sample_newest_first(void)
{
    static const int8_t expected_left[5] = {-71, -57, -42, -28, -14};
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;

    line_motion_init(&state);
    for (uint16_t i = 0u; i < 5u; ++i) {
        input.left_speed_mm_s = (int16_t)(50 * (i + 1u));
        input.right_speed_mm_s = (int16_t)(20 * (i + 1u));
        line_motion_step(&state, &input, &output);
    }

    input.obstacle_valid = true;
    input.distance_cm = 5u;
    line_motion_step(&state, &input, &output);
    input.elapsed_ms = LINE_BRAKE_MS;
    line_motion_step(&state, &input, &output);
    input.elapsed_ms = LINE_RECORD_PERIOD_MS;
    for (uint16_t i = 0u; i < 5u; ++i) {
        line_motion_step(&state, &input, &output);
        CHECK(output.left_percent == expected_left[i]);
    }

    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_WAIT_CLEAR);
    CHECK(output.left_percent == 0);
    CHECK(output.right_percent == 0);
}

static void test_wait_for_clear_and_confirm_for_one_second(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;
    unsigned int i;

    line_motion_init(&state);
    populate_history(&state);
    input.obstacle_valid = true;
    input.distance_cm = 2u;
    line_motion_step(&state, &input, &output);
    input.elapsed_ms = 100u;
    line_motion_step(&state, &input, &output);
    input.elapsed_ms = 20u;
    for (i = 0; i < LINE_HISTORY_SAMPLES; ++i) {
        line_motion_step(&state, &input, &output);
    }
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_WAIT_CLEAR);
    CHECK(output.left_percent == 0);
    CHECK(output.right_percent == 0);

    input.distance_cm = 20u;
    for (i = 0; i < 49; ++i) {
        line_motion_step(&state, &input, &output);
    }
    CHECK(state.phase == LINE_PHASE_WAIT_CLEAR);
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_NORMAL);
    CHECK(!output.obstacle_latched);

    input.distance_cm = 5u;
    for (i = 0; i < 50; ++i) {
        line_motion_step(&state, &input, &output);
    }
    CHECK(state.phase == LINE_PHASE_WAIT_CLEAR);
}

static void test_rewind_command_holds_across_ten_ms_ticks(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;
    int8_t held_left;
    int8_t held_right;

    line_motion_init(&state);
    populate_history(&state);
    input.obstacle_valid = true;
    input.distance_cm = 4u;
    line_motion_step(&state, &input, &output);
    input.elapsed_ms = 100u;
    line_motion_step(&state, &input, &output);

    input.elapsed_ms = 10u;
    line_motion_step(&state, &input, &output);
    line_motion_step(&state, &input, &output);
    held_left = output.left_percent;
    held_right = output.right_percent;
    CHECK(held_left < 0);
    CHECK(held_right < 0);

    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == held_left);
    CHECK(output.right_percent == held_right);
}

static void test_last_rewind_sample_holds_for_full_twenty_ms(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;
    uint8_t non_zero_ticks = 0u;

    line_motion_init(&state);
    state.phase = LINE_PHASE_REWIND;
    state.history_head = 1u;
    state.history_count = 1u;
    state.history[0].left_speed_mm_s = LINE_MAX_SPEED_MM_S;
    state.history[0].right_speed_mm_s = LINE_MAX_SPEED_MM_S;
    state.replay_remaining = 1u;
    input.elapsed_ms = 10u;
    input.obstacle_valid = true;
    input.distance_cm = LINE_OBSTACLE_STOP_CM;

    for (uint8_t i = 0u; i < 4u; ++i) {
        line_motion_step(&state, &input, &output);
        if (output.left_percent != 0 || output.right_percent != 0) {
            ++non_zero_ticks;
        }
    }
    CHECK(non_zero_ticks == 2u);
    CHECK(state.phase == LINE_PHASE_WAIT_CLEAR);
}

static void test_disabled_line_resets_motion(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;

    line_motion_init(&state);
    populate_history(&state);
    input.obstacle_valid = true;
    input.distance_cm = 3u;
    line_motion_step(&state, &input, &output);

    input.enabled = false;
    line_motion_step(&state, &input, &output);
    CHECK(state.phase == LINE_PHASE_IDLE);
    CHECK(state.history_count == 0u);
    CHECK(output.left_percent == 0);
    CHECK(output.right_percent == 0);
}

static void test_line_runs_without_ultrasonic(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;
    line_motion_init(&state);
    input.obstacle_valid = false;
    input.distance_cm = 0u;
    for (int i = 0; i < 100; ++i) {
        line_motion_step(&state, &input, &output);
        CHECK(output.left_percent > 0 && output.right_percent > 0);
        CHECK(!output.obstacle_latched);
    }
    input.obstacle_valid = true;
    input.distance_cm = 5u;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 0 && output.right_percent == 0);
    CHECK(output.obstacle_latched);
}

static void test_vision_assist_preserves_local_control(void)
{
    line_motion_t state;
    line_motion_input_t input = normal_input();
    line_motion_output_t output;
    line_motion_init(&state);
    input.speed_percent = 15u;
    input.obstacle_valid = true;
    input.distance_cm = 35u;
    input.vision_bias_percent = 4;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 19 && output.right_percent == 11);

    input.vision_bias_percent = -4;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 11 && output.right_percent == 19);

    input.vision_bias_percent = 6;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 15 && output.right_percent == 15);

    input.vision_bias_percent = 4;
    input.distance_cm = 25u;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 15 && output.right_percent == 15);
    input.obstacle_valid = false;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 15 && output.right_percent == 15);

    input.obstacle_valid = true;
    input.distance_cm = 35u;
    input.speed_percent = 30u;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 30 && output.right_percent == 30);
    input.speed_percent = 15u;
    input.ccd_valid = false;
    line_motion_step(&state, &input, &output);
    CHECK(output.left_percent == 0 && output.right_percent == 0);

    input.ccd_valid = true;
    input.distance_cm = 5u;
    line_motion_step(&state, &input, &output);
    CHECK(output.obstacle_latched && output.left_percent == 0 && output.right_percent == 0);
}

int main(void)
{
    test_ccd_center_and_right();
    test_ccd_rejects_flat_frame();
    test_lf04_error();
    test_optional_ultrasonic_guard();
    test_pan_angle_uses_actual_step_position();
    test_ccd_is_primary_and_lf04_supplements();
    test_lf04_center_two_sensors_are_still_a_line();
    test_obstacle_brakes_then_replays_newest_first();
    test_obstacle_replays_every_sample_newest_first();
    test_wait_for_clear_and_confirm_for_one_second();
    test_rewind_command_holds_across_ten_ms_ticks();
    test_last_rewind_sample_holds_for_full_twenty_ms();
    test_disabled_line_resets_motion();
    test_line_runs_without_ultrasonic();
    test_vision_assist_preserves_local_control();
    printf("checks=%d failures=%d\n", s_checks, s_failures);
    return s_failures == 0 ? 0 : 1;
}
