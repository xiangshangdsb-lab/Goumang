#include "vision.h"

#include <string.h>

#include "app/app_config.h"
#include "app/app_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rgb565_format.h"

#define VISION_CELL_SIZE 8
#define VISION_MAX_GRID_W 128
#define VISION_MAX_GRID_H 80

static const char *TAG = "vision";

static uint32_t s_frame_counter;
static uint32_t s_fps_counter;
static int64_t s_fps_since_us;
static td_vision_result_t s_result;

static bool is_dark_cell(const uint8_t *rgb,
                         uint32_t width,
                         uint32_t height,
                         uint32_t cell_x,
                         uint32_t cell_y,
                         uint8_t threshold)
{
    uint32_t start_x = cell_x * VISION_CELL_SIZE;
    uint32_t start_y = cell_y * VISION_CELL_SIZE;
    uint32_t end_x = start_x + VISION_CELL_SIZE;
    uint32_t end_y = start_y + VISION_CELL_SIZE;
    uint32_t dark = 0;
    uint32_t samples = 0;

    if (end_x > width) {
        end_x = width;
    }
    if (end_y > height) {
        end_y = height;
    }

    for (uint32_t y = start_y; y < end_y; y += 2) {
        const uint8_t *row = rgb + (size_t)y * width * 2u;
        for (uint32_t x = start_x; x < end_x; x += 2) {
            const uint8_t *ptr = row + (size_t)x * 2u;
            uint8_t r;
            uint8_t g;
            uint8_t b;
            uint8_t gray;
            uint16_t pixel = td_rgb565_read_le(ptr);

            td_rgb565_to_rgb8(pixel, &r, &g, &b);
            gray = (uint8_t)((r * 77u + g * 150u + b * 29u) >> 8);
            ++samples;

            if (gray < threshold) {
                ++dark;
            }
        }
    }

    return samples > 0u && dark >= (samples * 5u + 9u) / 10u;
}

void vision_process_frame(const uint8_t *rgb565,
                          uint32_t width,
                          uint32_t height)
{
    if (rgb565 == NULL || width < VISION_CELL_SIZE ||
        height < VISION_CELL_SIZE) {
        return;
    }
    if ((++s_frame_counter % 2u) != 0u) {
        return;
    }

    uint32_t grid_w =
        (width + VISION_CELL_SIZE - 1u) / VISION_CELL_SIZE;
    uint32_t grid_h =
        (height + VISION_CELL_SIZE - 1u) / VISION_CELL_SIZE;
    if (grid_w > VISION_MAX_GRID_W || grid_h > VISION_MAX_GRID_H) {
        return;
    }

    uint8_t threshold;
    app_state_lock();
    threshold = app_state_mutable()->threshold;
    app_state_unlock();

    memset(&s_result, 0, sizeof(s_result));
    uint64_t x_sum = 0;
    uint32_t dark_cells = 0;
    uint32_t line_start = grid_h / 2u;
    uint16_t cells[TD_LINE_IMAGE_FEATURES] = {0};
    uint16_t dark[TD_LINE_IMAGE_FEATURES] = {0};

    /* 汇总相机下半幅三个距离带的暗色分布，复用巡线检测的采样。 */
    for (uint32_t gy = line_start; gy < grid_h; ++gy) {
        uint32_t band = (gy - line_start) * TD_LINE_IMAGE_ROWS / (grid_h - line_start);
        for (uint32_t gx = 0; gx < grid_w; ++gx) {
            uint32_t bin = band * TD_LINE_IMAGE_COLS + gx * TD_LINE_IMAGE_COLS / grid_w;
            ++cells[bin];
            if (is_dark_cell(rgb565, width, height, gx, gy, threshold)) {
                ++dark[bin];
                x_sum += gx * VISION_CELL_SIZE +
                         VISION_CELL_SIZE / 2u;
                ++dark_cells;
            }
        }
    }

    if (dark_cells >= 8u) {
        uint32_t centroid = (uint32_t)(x_sum / dark_cells);
        int32_t offset = (int32_t)centroid - (int32_t)(width / 2u);
        int32_t error = offset * 100 / (int32_t)(width / 2u);
        if (error > 100) {
            error = 100;
        } else if (error < -100) {
            error = -100;
        }
        s_result.line.line_found = true;
        s_result.line.centroid_x = (uint16_t)centroid;
        s_result.line.error = (int16_t)error;
        s_result.line.dark_pixels = dark_cells;
        s_result.line.confidence =
            (uint8_t)(dark_cells * 100u /
                      (grid_w * (grid_h - line_start) / 2u + 1u));
        if (s_result.line.confidence > 100u) {
            s_result.line.confidence = 100u;
        }
    }

    ++s_fps_counter;
    int64_t now_us = esp_timer_get_time();
    uint16_t fps_x10 = 0;
    if (s_fps_since_us == 0) {
        s_fps_since_us = now_us;
    } else if (now_us - s_fps_since_us >= 1000000) {
        fps_x10 = (uint16_t)((int64_t)s_fps_counter * 10000000 /
                             (now_us - s_fps_since_us));
        s_fps_counter = 0;
        s_fps_since_us = now_us;
    }

    app_state_lock();
    td_status_t *status = app_state_mutable();
    status->vision_ok = true;
    status->vision = s_result;
    status->line = s_result.line;

    /* 同一把锁内读取 CCD、轮速及其接收时刻，避免 HTTP 轮询错配标签。 */
    td_line_sample_t sample = {.frame_us = now_us};
    const uint8_t required = RP_LINE_FLAG_ACTIVE | RP_LINE_FLAG_CCD_FOUND |
                             RP_LINE_FLAG_CCD_VALID;
    const uint8_t recovery = RP_LINE_FLAG_OBSTACLE | RP_LINE_FLAG_REWIND |
                             RP_LINE_FLAG_WAIT_CLEAR;
    int32_t speed = ((int32_t)status->chassis.left_speed_mm_s +
                     status->chassis.right_speed_mm_s) / 2;
    sample.valid = status->stm32_usb_connected && status->mode == TD_MODE_LINE &&
        (status->line_state.flags & required) == required &&
        (status->line_state.flags & recovery) == 0 &&
        status->line_state.ccd_center < 128u && status->line_state.ccd_confidence > 0 &&
        app_rx_age_ms(now_us, status->stm32_line_rx_us) <= 160u &&
        app_rx_age_ms(now_us, status->stm32_chassis_rx_us) <= 160u &&
        app_rx_age_ms(now_us, status->stm32_motor_rx_us) <= 160u &&
        (status->motor.pan_flags & (RP_PAN_MOVING | RP_PAN_BOOT_RETURN)) == 0 &&
        (!(status->motor.pan_flags & RP_PAN_KNOWN) ||
         (status->motor.pan_deg >= RP_PAN_HOME_DEG - 5 &&
          status->motor.pan_deg <= RP_PAN_HOME_DEG + 5)) &&
        speed >= 20 && speed <= 500;
    if (sample.valid) {
        sample.ccd_us = status->stm32_line_rx_us;
        sample.ccd_center = status->line_state.ccd_center;
        sample.ccd_confidence = status->line_state.ccd_confidence;
        sample.speed_mm_s = (int16_t)speed;
        for (unsigned i = 0; i < TD_LINE_IMAGE_FEATURES; ++i) {
            sample.dark[i] = cells[i] ? (uint8_t)(dark[i] * 100u / cells[i]) : 0;
        }
#if TD_LINE_MODEL_AVAILABLE
        sample.prediction = td_line_predict(&sample);
#endif
    }
    status->line_learning = sample;
    ++status->vision_frames;
    if (fps_x10 != 0u) {
        status->vision_fps_x10 = fps_x10;
    }
    app_state_unlock();
}

esp_err_t vision_start(void)
{
    ESP_LOGI(TAG, "black-line detector ready");
    return ESP_OK;
}

void vision_set_threshold(uint8_t threshold)
{
    if (threshold < 10u) {
        threshold = 10u;
    } else if (threshold > 200u) {
        threshold = 200u;
    }
    app_state_lock();
    app_state_mutable()->threshold = threshold;
    app_state_unlock();
}
