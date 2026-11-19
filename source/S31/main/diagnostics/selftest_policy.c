#include "selftest_policy.h"
#include <stddef.h>
#include <string.h>
#include "protocol/robot_protocol.h"

static const struct { const char *id, *name, *clip; } s_modules[] = {
    {"storage", "配置存储", "st_storage"},
    {"display", "显示屏", "st_display"},
    {"touch", "触摸屏", "st_touch"},
    {"keys", "按键", "st_keys"},
    {"audio", "语音输出", "st_audio"},
    {"camera", "摄像头", "st_camera"},
    {"face", "人脸模型", "st_face"},
    {"objects", "物体识别模型", "st_objects"},
    {"wifi", "无线网络", "st_wifi"},
    {"web", "网页控制服务", "st_web"},
    {"agent", "智能助手", "st_agent"},
    {"chassis", "底盘通信", "st_chassis"},
    {"battery", "电池电压", "st_battery"},
    {"imu", "惯性传感器", "st_imu"},
    {"ultrasonic", "超声波测距", "st_ultra"},
    {"ccd", "线性巡线传感器", "st_ccd"},
    {"infrared", "四路巡线", "st_infrared"},
    {"motors", "电机与编码器", "st_motors"},
    {"pan", "云台", "st_pan"},
    {"safety", "底盘安全状态", "st_safety"},
};
static const struct { const char *id, *name, *clip; } s_results[] = {
    {"init_ok", "初始化正常", "st_init_ok"},
    {"data_ok", "数据正常", "st_data_ok"},
    {"signal_ok", "已检测到巡线信号", "st_signal_ok"},
    {"fault", "异常", "st_fault"},
    {"not_ready", "未就绪", "st_not_ready"},
    {"no_data", "没有有效数据", "st_no_data"},
    {"disconnected", "未连接", "st_disconnected"},
    {"manual_check", "需要人工确认", "st_manual"},
    {"optional_no_echo", "无有效回波，可继续运行", "st_no_echo"},
    {"needs_calibration", "需要标定", "st_calibrate"},
    {"estop", "急停已触发", "st_estop"},
    {"low_battery", "电压偏低", "st_low_battery"},
    {"network_up", "已联网", "st_net_up"},
    {"access_point", "热点已开启", "st_net_ap"},
    {"connecting", "连接中", "st_net_wait"},
    {"configured", "已配置", "st_configured"},
    {"unconfigured", "未配置", "st_unconfigured"},
    {"skipped", "底盘未连接，已跳过", "st_skipped"},
};

const char *st_module_id(st_module_t m) { return (unsigned)m < ST_ITEM_COUNT ? s_modules[m].id : "unknown"; }
const char *st_module_name(st_module_t m) { return (unsigned)m < ST_ITEM_COUNT ? s_modules[m].name : "未知"; }
const char *st_module_clip(st_module_t m) { return (unsigned)m < ST_ITEM_COUNT ? s_modules[m].clip : NULL; }
#define ST_RESULT_COUNT (sizeof(s_results) / sizeof(s_results[0]))
const char *st_result_id(st_result_t r) { return (unsigned)r < ST_RESULT_COUNT ? s_results[r].id : "unknown"; }
const char *st_result_name(st_result_t r) { return (unsigned)r < ST_RESULT_COUNT ? s_results[r].name : "未知"; }
const char *st_result_clip(st_result_t r) { return (unsigned)r < ST_RESULT_COUNT ? s_results[r].clip : NULL; }
bool st_is_error(st_result_t r) { return r == ST_FAULT || r == ST_ESTOP || r == ST_LOW_BATTERY; }
bool st_needs_attention(st_result_t r)
{
    return st_is_error(r) || r == ST_NOT_READY || r == ST_NO_DATA ||
           r == ST_DISCONNECTED || r == ST_MANUAL || r == ST_CALIBRATE ||
           r == ST_UNCONFIGURED || r == ST_NET_WAIT;
}

void st_evaluate(const st_inputs_t *i, st_results_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    for (int m = 0; m < ST_ITEM_COUNT; ++m) out->results[m] = ST_NOT_READY;
    if (!i) { out->attention = ST_ITEM_COUNT; return; }
    st_result_t *r = out->results;
    r[ST_STORAGE] = i->storage ? ST_INIT_OK : ST_FAULT;
    r[ST_DISPLAY] = i->display ? ST_INIT_OK : ST_FAULT;
    r[ST_TOUCH] = i->touch ? ST_INIT_OK : ST_FAULT;
    r[ST_KEYS] = i->keys ? ST_INIT_OK : ST_FAULT;
    r[ST_AUDIO] = i->audio ? ST_INIT_OK : ST_FAULT;
    r[ST_CAMERA] = !i->camera_init ? ST_FAULT : i->camera_frame ? ST_DATA_OK : ST_NO_DATA;
    r[ST_FACE] = !i->face_init ? ST_FAULT : i->face_ready ? ST_INIT_OK : ST_NOT_READY;
    r[ST_OBJECTS] = !i->objects_init ? ST_FAULT : i->objects_ready ? ST_INIT_OK : ST_NOT_READY;
    r[ST_WIFI] = !i->wifi_init ? ST_FAULT : i->wifi_up ? ST_NET_UP : i->wifi_ap ? ST_NET_AP : ST_NET_WAIT;
    r[ST_WEB] = i->web ? ST_INIT_OK : ST_FAULT;
    r[ST_AGENT] = !i->agent_init ? ST_FAULT : i->agent_configured ? ST_CONFIGURED : ST_UNCONFIGURED;
    r[ST_CHASSIS] = !i->usb_init ? ST_FAULT : i->chassis_online ? ST_DATA_OK : ST_DISCONNECTED;
    for (int m = ST_BATTERY; m < ST_ITEM_COUNT; ++m) r[m] = ST_SKIPPED;
    if (i->usb_init && i->chassis_online) {
        r[ST_BATTERY] = !i->sensor_fresh || i->battery_mv == 0u || i->battery_mv == UINT16_MAX ? ST_NO_DATA :
            i->battery_mv < 10000u || (i->safety_fresh && (i->faults & RP_FAULT_LOW_BATTERY)) ? ST_LOW_BATTERY : ST_DATA_OK;
        r[ST_IMU] = !i->imu_fresh || !i->safety_fresh ? ST_NO_DATA :
            (i->faults & RP_FAULT_IMU_OFFLINE) ? ST_FAULT : ST_DATA_OK;
        r[ST_ULTRA] = !i->sensor_fresh ? ST_NO_DATA :
            i->distance_cm == 0u || i->distance_cm == UINT16_MAX ? ST_NO_ECHO : ST_DATA_OK;
        r[ST_CCD] = !i->line_fresh || !(i->line_flags & RP_LINE_FLAG_CCD_VALID) ? ST_NO_DATA :
            (i->line_flags & RP_LINE_FLAG_CCD_FOUND) ? ST_SIGNAL_OK : ST_MANUAL;
        /* Static telemetry cannot prove four infrared sensors or motor motion. */
        r[ST_INFRARED] = i->line_fresh ? ST_MANUAL : ST_NO_DATA;
        r[ST_MOTORS] = i->motor_fresh ? ST_MANUAL : ST_NO_DATA;
        r[ST_PAN] = !i->motor_fresh || !i->safety_fresh ? ST_NO_DATA :
            (i->faults & RP_FAULT_GIMBAL_UNHOMED) ? ST_CALIBRATE : ST_INIT_OK;
        r[ST_SAFETY] = !i->safety_fresh ? ST_NO_DATA :
            (i->faults & RP_FAULT_ESTOP) ? ST_ESTOP :
            (i->faults & RP_FAULT_LOW_BATTERY) ? ST_LOW_BATTERY :
            (i->faults & RP_FAULT_UART_OVERFLOW) ? ST_FAULT : ST_INIT_OK;
    }
    for (int m = 0; m < ST_ITEM_COUNT; ++m) {
        if (st_is_error(r[m])) ++out->errors;
        if (st_needs_attention(r[m])) ++out->attention;
    }
}
