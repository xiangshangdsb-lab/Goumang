#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** Initialize the DVP camera, LCD preview, web snapshot, and vision feeds. */
esp_err_t camera_app_start(void);

/** Update live/stalled camera state from the real frame timestamp. */
bool camera_app_has_fresh_frame(void);

/** Return a 480x360, 24-bit BMP snapshot allocated from PSRAM. */
esp_err_t camera_app_get_bmp(uint8_t **data, size_t *length);
void camera_app_free_bmp(uint8_t *data);

/** Return a PSRAM JPEG from a frame captured at or after min_capture_us. */
esp_err_t camera_app_get_jpeg(int64_t min_capture_us, uint8_t **data, size_t *length);
