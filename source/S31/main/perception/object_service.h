#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TD_MAX_OBJECTS 8
#define TD_OBJECT_NAME_LEN 32

typedef struct {
    uint8_t category;
    uint8_t confidence;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    char name[TD_OBJECT_NAME_LEN];
} td_object_t;

typedef struct {
    bool ready;
    uint8_t count;
    uint32_t inference_count;
    td_object_t items[TD_MAX_OBJECTS];
} td_object_status_t;

esp_err_t object_service_start(void);
void object_service_submit_frame(const uint8_t *rgb565,
                                 uint32_t width,
                                 uint32_t height,
                                 bool swapped);
void object_service_snapshot(td_object_status_t *out);

#ifdef __cplusplus
}
#endif
