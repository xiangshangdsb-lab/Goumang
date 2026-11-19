#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Mount the "voice" SPIFFS partition and init the ES8389 speaker path. */
esp_err_t voice_init(void);

/** Blocking: play /voice/<name>.wav to the end. */
esp_err_t voice_play(const char *name);

/** Play a module name and result without another prompt interleaving. */
esp_err_t voice_play_pair(const char *first, const char *second);

/** Non-blocking: queue a clip for the voice worker task (safe from HTTP). */
esp_err_t voice_play_async(const char *name);

/** Queue a complete greeting, with the registered face ID read digit by digit. */
esp_err_t voice_greet_async(uint16_t face_id);

typedef struct {
    bool ready;
    bool playing;
    esp_err_t last_error;
    uint32_t completed;
    char clip[24];
} voice_status_t;

void voice_get_status(voice_status_t *out);
bool voice_ready(void);

#ifdef __cplusplus
}
#endif
