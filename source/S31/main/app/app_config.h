#pragma once

/*
 * Goumang on ESP32-S31-Korvo-1.
 * Ported from the legacy prototype; hardware-specific numbers follow the
 * ESP32-S31-Korvo-1 BSP (DVP OV3660/SC101IOT, 800x480 RGB LCD,
 * ES8389 codec).
 */
#define TD_FW_VERSION              "s31-v3.10.0-speed100"

/*
 * Chassis link: C30D UART3 USB-C (CH9102F bridge) to the S31 USB-A host
 * port. The wire protocol is unchanged from the legacy firmware.
 */
#define TD_USB_LINK_VID             0x1A86u
#define TD_USB_LINK_PID             0x55D4u
#define TD_USB_LINK_BAUD            115200u

/* DVP sensor captures 640x480; PPA scales directly to the LVGL preview size. */
#define TD_CAM_VIEW_WIDTH          480
#define TD_CAM_VIEW_HEIGHT         360

#define TD_AP_SSID                 "Goumang-S31"
#define TD_AP_SETUP_SSID           "Goumang-S31-Setup"
#define TD_AP_CHANNEL              6
#define TD_AP_MAX_CLIENTS          4
#define TD_AP_URL                  "http://192.168.4.1"
#define TD_MDNS_HOSTNAME           "goumang"

#define TD_CONTROL_PERIOD_MS       50
#define TD_DRIVE_PERIOD_MS         100
#define TD_STM32_ONLINE_MS         600
#define TD_MANUAL_HOLD_MS          350
#define TD_MANUAL_MAX_HOLD_MS      60000
#define TD_WEB_MANUAL_MAX_HOLD_MS  500

#define TD_LINE_MIN_CONFIDENCE     8
#define TD_LINE_STEER_NUM          3
#define TD_LINE_STEER_DEN          5
#define TD_LINE_SPEED              32
/* 默认允许视觉辅助。有实车权重才发视觉偏置，普通巡线入口用低速。 */
#define TD_LINE_ASSIST_ENABLED      1
#define TD_LINE_MODULE_SPEED       (TD_LINE_ASSIST_ENABLED ? 15 : 30)
#define TD_USER_TRACK_MIN_PERCENT  15u
#define TD_USER_TRACK_MAX_PERCENT  100u
#define TD_USER_TRACK_STEP_PERCENT 5u
#define TD_LINE_LOST_STOP_MS       450
#define TD_INSPECT_PHASE_MS        1200

/* Voice prompt clips live in the "voice" SPIFFS partition (see voice/). */
#define TD_VOICE_DEFAULT_CLIP      "boot_ok"

/* The main display is 800x480. The camera preview occupies 480x360 and the
 * control console occupies the remaining 250 px on the right. */
#define TD_UI_WIDTH                800
#define TD_UI_HEIGHT               480
#define TD_CAMERA_PREVIEW_SCALE    224
#define TD_CAMERA_PREVIEW_W        (TD_CAM_VIEW_WIDTH * TD_CAMERA_PREVIEW_SCALE / 256)
#define TD_CAMERA_PREVIEW_H        (TD_CAM_VIEW_HEIGHT * TD_CAMERA_PREVIEW_SCALE / 256)
