#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ST_STORAGE, ST_DISPLAY, ST_TOUCH, ST_KEYS, ST_AUDIO, ST_CAMERA,
    ST_FACE, ST_OBJECTS, ST_WIFI, ST_WEB, ST_AGENT, ST_CHASSIS,
    ST_BATTERY, ST_IMU, ST_ULTRA, ST_CCD, ST_INFRARED, ST_MOTORS,
    ST_PAN, ST_SAFETY, ST_ITEM_COUNT
} st_module_t;

typedef enum {
    ST_INIT_OK, ST_DATA_OK, ST_SIGNAL_OK, ST_FAULT, ST_NOT_READY,
    ST_NO_DATA, ST_DISCONNECTED, ST_MANUAL, ST_NO_ECHO, ST_CALIBRATE,
    ST_ESTOP, ST_LOW_BATTERY, ST_NET_UP, ST_NET_AP, ST_NET_WAIT,
    ST_CONFIGURED, ST_UNCONFIGURED, ST_SKIPPED
} st_result_t;

typedef struct {
    bool storage, display, touch, keys, audio, camera_init, camera_frame;
    bool face_init, face_ready, objects_init, objects_ready;
    bool wifi_init, wifi_up, wifi_ap, web, agent_init, agent_configured;
    bool usb_init, chassis_online;
    bool sensor_fresh, imu_fresh, line_fresh, motor_fresh, safety_fresh;
    uint16_t battery_mv, distance_cm;
    uint8_t faults, line_flags;
} st_inputs_t;

typedef struct {
    st_result_t results[ST_ITEM_COUNT];
    uint8_t errors, attention;
} st_results_t;

void st_evaluate(const st_inputs_t *input, st_results_t *out);
const char *st_module_id(st_module_t module);
const char *st_module_name(st_module_t module);
const char *st_module_clip(st_module_t module);
const char *st_result_id(st_result_t result);
const char *st_result_name(st_result_t result);
const char *st_result_clip(st_result_t result);
bool st_needs_attention(st_result_t result);
bool st_is_error(st_result_t result);
