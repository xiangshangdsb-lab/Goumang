#include "line_follower.h"

#include <string.h>

void line_follow_default_config(line_follow_config_t *config)
{
    if (config == NULL) {
        return;
    }
    config->roi_top = 120;       /* task specification: rows 120..239 */
    config->roi_bottom = 239;
    config->threshold = 90;
    config->sample_step = 2;
    config->min_dark_pixels = 40;
    config->dark_is_line = true;
}

bool line_follow_compute_gray(const uint8_t *gray,
                              size_t stride,
                              size_t width,
                              size_t height,
                              const line_follow_config_t *config,
                              line_follow_result_t *result)
{
    size_t y;
    size_t x;
    size_t step;
    uint64_t x_sum = 0;
    uint32_t count = 0;
    uint32_t samples = 0;
    uint32_t confidence;
    int64_t offset;
    int64_t half_width;
    int64_t error;

    if (result == NULL) {
        return false;
    }
    memset(result, 0, sizeof(*result));

    if (gray == NULL || width < 2u || height == 0u ||
        config == NULL || config->sample_step == 0u ||
        config->roi_top >= config->roi_bottom ||
        config->roi_bottom >= height) {
        return false;
    }

    step = config->sample_step;
    for (y = config->roi_top; y <= config->roi_bottom; y += step) {
        const uint8_t *row = gray + y * stride;
        for (x = 0; x < width; x += step) {
            uint8_t pixel = row[x];
            bool is_dark = pixel < config->threshold;
            bool selected = config->dark_is_line ? is_dark : !is_dark;
            ++samples;
            if (selected) {
                x_sum += x;
                ++count;
            }
        }
    }

    result->dark_pixels = count;
    if (count < config->min_dark_pixels) {
        return true;
    }

    result->centroid_x = (uint16_t)(x_sum / count);
    result->line_found = true;

    half_width = (int64_t)(width / 2u);
    offset = (int64_t)result->centroid_x - (int64_t)(width / 2u);
    error = (offset * 100) / half_width;
    if (error > 100) {
        error = 100;
    } else if (error < -100) {
        error = -100;
    }
    result->error = (int16_t)error;

    confidence = count * 100u / (samples / 8u + 1u);
    result->confidence = (uint8_t)(confidence > 100u ? 100u : confidence);
    return true;
}
