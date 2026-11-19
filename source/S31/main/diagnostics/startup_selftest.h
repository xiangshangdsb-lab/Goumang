#pragma once
#include "esp_err.h"
#include "selftest_policy.h"

typedef struct {
    bool storage, display, touch, keys, camera, face, objects, wifi, usb, agent;
} startup_init_t;
typedef struct {
    bool running, ready, complete, announcing;
    uint8_t current;
    uint32_t checked_at_ms;
    esp_err_t audio_error;
    st_results_t checks;
} startup_report_t;

esp_err_t startup_selftest_start(const startup_init_t *init);
void startup_selftest_snapshot(startup_report_t *out);
void startup_selftest_log(void);
