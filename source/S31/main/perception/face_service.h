#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool ready;
    bool face_found;
    bool recognized;
    bool enrolling;
    uint16_t face_id;
    uint16_t registered_count;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint8_t similarity;
    uint32_t inference_count;
    char message[64];
} td_face_status_t;

esp_err_t face_service_start(void);
void face_service_submit_frame(const uint8_t *rgb565,
                               uint32_t width,
                               uint32_t height,
                               bool swapped);
esp_err_t face_service_request_enroll(void);
esp_err_t face_service_delete_last(void);
esp_err_t face_service_clear_all(void);
void face_service_snapshot(td_face_status_t *out);

#ifdef __cplusplus
}
#endif
