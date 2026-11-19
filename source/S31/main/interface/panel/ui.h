#pragma once

#include "esp_err.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Build the 800x480 status screen (call after bsp_display_start). */
esp_err_t ui_start(void);

/** Move the live camera canvas into the normal or face-registration view. */
void ui_attach_camera(lv_obj_t *canvas);

/** Clear camera-canvas references before the caller deletes the canvas. */
void ui_detach_camera(void);

/** Refresh dynamic labels; call from a low-priority task. */
void ui_refresh(void);

#ifdef __cplusplus
}
#endif
