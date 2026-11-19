#include "web_server.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <errno.h>
#include <string.h>

#include "agent/agent_service.h"
#include "agent/agent_runtime.h"
#include "app/app_config.h"
#include "app/app_state.h"
#include "cJSON.h"
#include "diagnostics/c30d_monitor.h"
#include "perception/camera_app.h"
#include "control/control.h"
#include "bus/robot_bus.h"
#include "app/robot_services.h"
#include "network/device_auth.h"
#include "network/device_auth_store.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "perception/face_service.h"
#include "perception/object_service.h"
#include "protocol/robot_protocol.h"
#include "perception/vision.h"
#include "audio/voice_player.h"
#include "diagnostics/startup_selftest.h"
#include "network/wifi_mgr.h"

static const char *TAG = "web";

extern const uint8_t dashboard_html_start[]
    asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[]
    asm("_binary_dashboard_html_end");

static bool query_int(httpd_req_t *req, const char *key, int fallback, int *out)
{
    char query[192];
    char value[24];
    esp_err_t err = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (err == ESP_OK) {
        err = httpd_query_key_value(query, key, value, sizeof(value));
    }
    if (err == ESP_ERR_NOT_FOUND) {
        *out = fallback;
        return true;
    }
    if (err != ESP_OK) {
        return false;
    }
    char *end;
    errno = 0;
    long number = strtol(value, &end, 10);
    if (end == value || *end != '\0' || errno == ERANGE ||
        number < INT_MIN || number > INT_MAX) {
        return false;
    }
    *out = (int)number;
    return true;
}

static void query_str(httpd_req_t *req, const char *key, char *out,
                      size_t out_len, const char *fallback)
{
    char query[192];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, key, out, out_len) != ESP_OK) {
        strlcpy(out, fallback, out_len);
    }
}

static bool safe_voice_name(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return false;
    }
    for (const char *p = name; *p != '\0'; ++p) {
        if (!(isalnum((unsigned char)*p) || *p == '_' || *p == '-')) {
            return false;
        }
    }
    return true;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static bool require_device_auth(httpd_req_t *req)
{
    char supplied[TD_AUTH_CODE_LEN + 4u];

    if (httpd_req_get_hdr_value_str(req, "X-TD-Token",
                                    supplied, sizeof(supplied)) == ESP_OK &&
        device_auth_code_matches(device_auth_control_code(), supplied)) {
        return true;
    }
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "TDToken");
    (void)httpd_resp_sendstr(
        req, "{\"ok\":false,\"error\":\"unauthorized\"}");
    return false;
}

static esp_err_t read_json_body(httpd_req_t *req, char *buffer,
                                size_t capacity, cJSON **json)
{
    size_t received = 0u;
    int remaining;

    if (buffer == NULL || capacity < 2u || json == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (req->content_len <= 0 ||
        (size_t)req->content_len >= capacity) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_ERR_INVALID_SIZE;
    }
    remaining = req->content_len;
    while (remaining > 0) {
        int ret = httpd_req_recv(req, buffer + received,
                                 (size_t)remaining);
        if (ret <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                "body receive failed");
            return ESP_FAIL;
        }
        received += (size_t)ret;
        remaining -= ret;
    }
    buffer[received] = '\0';
    *json = memchr(buffer, 0, received) == NULL ?
            cJSON_ParseWithOpts(buffer, NULL, true) : NULL;
    if (*json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid JSON");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t handle_root(httpd_req_t *req)
{
    size_t length = (size_t)(dashboard_html_end - dashboard_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)dashboard_html_start, length);
}

static esp_err_t handle_status(httpd_req_t *req)
{
    td_status_t status;
    td_face_status_t face;
    td_object_status_t objects;
    const size_t json_capacity = 6144;
    char *json = heap_caps_malloc(
        json_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    app_state_snapshot(&status);
    face_service_snapshot(&face);
    object_service_snapshot(&objects);

    int written = snprintf(
        json, json_capacity,
        "{"
        "\"fw\":\"%s\","
        "\"mode\":\"%s\","
        "\"uptime_ms\":%llu,"
        "\"camera\":{\"ready\":%s,\"streaming\":%s},"
        "\"wifi\":{\"ap\":%s,\"client\":%s,\"station\":%s,"
        "\"setup\":%s,\"ip\":\"%s\",\"url\":\"%s\"},"
        "\"vision\":{\"ok\":%s,\"fps\":%u.%u,\"frames\":%lu,"
        "\"line\":%s,\"error\":%d,\"confidence\":%u,\"threshold\":%u},"
        "\"face\":{\"ready\":%s,\"found\":%s,\"recognized\":%s,"
        "\"enrolling\":%s,\"id\":%u,\"count\":%u,\"similarity\":%u,"
        "\"inferences\":%lu,"
        "\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u,"
        "\"message\":\"%s\"},"
        "\"motion\":{\"online\":%s,\"state\":%u,\"faults\":%u,"
        "\"left\":%d,\"right\":%d,\"yaw\":%.2f,"
        "\"distance\":%u,\"battery\":%u,"
        "\"top\":%u,\"bottom\":%u,"
        "\"line_bits\":%u,\"line_valid\":%s,\"line_enabled\":%s,"
        "\"control_source\":%u,\"line_flags\":%u,\"line_source\":%u,"
        "\"ccd_center\":%u,\"ccd_confidence\":%u,"
        "\"line_error\":%d,\"rewind_progress\":%u,"
        "\"gyro_ddps\":[%d,%d,%d],"
        "\"accel_mg\":[%d,%d,%d],"
        "\"motor\":{\"encoder\":[%d,%d],\"pwm\":[%d,%d],"
        "\"target_mm_s\":[%d,%d],\"pan\":%u,\"pan_flags\":%u,\"source\":%u,"
        "\"left_command\":%d,\"right_command\":%d}},"
        "\"out\":{\"left\":%d,\"right\":%d,\"pan\":%u,\"push\":%u,"
        "\"inspect_phase\":%u},"
        "\"objects\":{\"ready\":%s,\"count\":%u,\"inferences\":%lu,"
        "\"items\":[",
        TD_FW_VERSION,
        td_mode_name(status.mode),
        (unsigned long long)(app_now_us() / 1000),
        status.camera_ready ? "true" : "false",
        status.camera_streaming ? "true" : "false",
        status.ap_started ? "true" : "false",
        status.ap_client_connected ? "true" : "false",
        status.sta_connected ? "true" : "false",
        status.setup_ap ? "true" : "false",
        status.ap_ip,
        status.url,
        status.vision_ok ? "true" : "false",
        status.vision_fps_x10 / 10u,
        status.vision_fps_x10 % 10u,
        (unsigned long)status.vision_frames,
        status.line.line_found ? "true" : "false",
        status.line.error,
        status.line.confidence,
        status.threshold,
        face.ready ? "true" : "false",
        face.face_found ? "true" : "false",
        face.recognized ? "true" : "false",
        face.enrolling ? "true" : "false",
        face.face_id,
        face.registered_count,
        face.similarity,
        (unsigned long)face.inference_count,
        face.x,
        face.y,
        face.width,
        face.height,
        face.message,
        status.stm32_online ? "true" : "false",
        status.safety.robot_state,
        status.safety.fault_flags,
        status.chassis.left_speed_mm_s,
        status.chassis.right_speed_mm_s,
        (double)status.chassis.yaw_cdeg / 100.0,
        status.sensor.distance_cm,
        status.sensor.battery_mv,
        status.sensor.push_top_limit,
        status.sensor.push_bottom_limit,
        status.sensor.push_top_limit & 0x0Fu,
        (status.line_state.flags & RP_LINE_FLAG_ACTIVE) ?
            "true" : "false",
        (status.sensor.push_bottom_limit & 0x80u) ? "true" : "false",
        (status.sensor.push_bottom_limit >> 1u) & 0x07u,
        status.line_state.flags,
        status.line_state.source,
        status.line_state.ccd_center,
        status.line_state.ccd_confidence,
        status.line_state.line_error,
        status.line_state.rewind_progress,
        status.imu.gyro_x_ddps,
        status.imu.gyro_y_ddps,
        status.imu.gyro_z_ddps,
        status.imu.accel_x_mg,
        status.imu.accel_y_mg,
        status.imu.accel_z_mg,
        status.motor.encoder_delta_a,
        status.motor.encoder_delta_b,
        status.motor.pwm_a,
        status.motor.pwm_b,
        status.motor.target_a_mm_s,
        status.motor.target_b_mm_s,
        status.motor.pan_deg,
        status.motor.pan_flags,
        status.motor.control_source,
        status.motor.left_command,
        status.motor.right_command,
        status.out_left,
        status.out_right,
        status.pan_deg,
        status.push_action,
        status.inspect_phase,
        objects.ready ? "true" : "false",
        objects.count,
        (unsigned long)objects.inference_count);

    if (written < 0 || (size_t)written >= json_capacity) {
        heap_caps_free(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t used = (size_t)written;
    for (uint8_t i = 0; i < objects.count && i < TD_MAX_OBJECTS; ++i) {
        const td_object_t *item = &objects.items[i];
        int added = snprintf(
            json + used, json_capacity - used,
            "%s{\"name\":\"%s\",\"confidence\":%u,\"category\":%u,"
            "\"x\":%u,\"y\":%u,\"width\":%u,\"height\":%u}",
            i == 0u ? "" : ",",
            item->name, item->confidence, item->category,
            item->x, item->y, item->width, item->height);
        if (added < 0 || (size_t)added >= json_capacity - used) {
            heap_caps_free(json);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        used += (size_t)added;
    }

    voice_status_t voice;
    voice_get_status(&voice);
    int tail = snprintf(
        json + used, json_capacity - used,
        "],\"note\":\"COCO 80 类，置信度门槛 60%%\"},"
        "\"voice\":{\"ready\":%s,\"playing\":%s,\"last_error\":%d,"
        "\"completed\":%lu,\"clip\":\"%s\",\"partition\":\"voice\"}"
        "}",
        voice.ready ? "true" : "false", voice.playing ? "true" : "false",
        (int)voice.last_error, (unsigned long)voice.completed, voice.clip);
    if (tail < 0 || (size_t)tail >= json_capacity - used) {
        heap_caps_free(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    used += (size_t)tail;
    esp_err_t err = send_json(req, json);
    heap_caps_free(json);
    return err;
}

static esp_err_t handle_say(httpd_req_t *req)
{
    char clip[24];
    char json[128];
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    query_str(req, "clip", clip, sizeof(clip), TD_VOICE_DEFAULT_CLIP);
    if (!safe_voice_name(clip)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid clip");
        return ESP_FAIL;
    }
    esp_err_t err = voice_play_async(clip);
    snprintf(json, sizeof(json),
             "{\"ok\":%s,\"clip\":\"%s\"}",
             err == ESP_OK ? "true" : "false", clip);
    return send_json(req, json);
}

static esp_err_t handle_tune(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    int threshold;
    char json[96];
    if (query_int(req, "threshold", -1, &threshold) && threshold >= 10 && threshold <= 200) {
        vision_set_threshold((uint8_t)threshold);
    }
    td_status_t status;
    app_state_snapshot(&status);
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"threshold\":%u}", status.threshold);
    return send_json(req, json);
}

static esp_err_t handle_start(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    int speed;
    if (!query_int(req, "speed", TD_LINE_MODULE_SPEED, &speed) ||
        speed < (int)TD_USER_TRACK_MIN_PERCENT || speed > (int)TD_USER_TRACK_MAX_PERCENT) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid track speed");
        return ESP_FAIL;
    }
    bool ok = control_start_line_speed((uint8_t)speed);
    if (ok) {
        voice_play_async("line_follow");
    }
    return send_json(req, ok ?
                     "{\"ok\":true,\"mode\":\"LINE\"}" :
                     "{\"ok\":false,\"error\":\"C30D did not acknowledge\"}");
}

static esp_err_t handle_stop(httpd_req_t *req)
{
    (void)req;
    bool ok = control_halt();
    return send_json(req, ok ?
                     "{\"ok\":true,\"mode\":\"IDLE\"}" :
                     "{\"ok\":false,\"error\":\"C30D did not acknowledge\"}");
}

static esp_err_t handle_estop(httpd_req_t *req)
{
    (void)req;
    bool ok = control_estop();
    voice_play_async("obstacle");
    return send_json(req, ok ?
                     "{\"ok\":true,\"mode\":\"ESTOP\"}" :
                     "{\"ok\":false,\"mode\":\"ESTOP\",\"error\":\"C30D did not acknowledge\"}");
}

static esp_err_t handle_clear(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    bool ok = control_clear_estop();
    return send_json(req, ok ?
                     "{\"ok\":true,\"mode\":\"IDLE\"}" :
                     "{\"ok\":false,\"error\":\"C30D did not acknowledge\"}");
}

static esp_err_t handle_inspect(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    bool ok = control_start_inspect();
    voice_play_async("inspect");
    return send_json(req, ok ?
                     "{\"ok\":true,\"mode\":\"INSPECT\"}" :
                     "{\"ok\":false,\"error\":\"C30D offline or pan uncalibrated\"}");
}

static esp_err_t handle_move(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    int left;
    int right;
    int hold_ms;
    if (!query_int(req, "left", 0, &left) ||
        !query_int(req, "right", 0, &right) ||
        !query_int(req, "ms", TD_MANUAL_HOLD_MS, &hold_ms) ||
        left < -100 || left > 100 || right < -100 || right > 100) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid speed");
        return ESP_FAIL;
    }
    if (hold_ms < 100) {
        hold_ms = TD_MANUAL_HOLD_MS;
    } else if (hold_ms > TD_WEB_MANUAL_MAX_HOLD_MS) {
        hold_ms = TD_WEB_MANUAL_MAX_HOLD_MS;
    }
    bool ok = control_manual((int8_t)left, (int8_t)right,
                             (uint32_t)hold_ms);
    return send_json(req, ok ?
                     "{\"ok\":true,\"mode\":\"MANUAL\"}" :
                     "{\"ok\":false,\"error\":\"C30D offline or rejected\"}");
}

static esp_err_t handle_actuator(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    int pan;
    if (!query_int(req, "pan", -1, &pan) || pan < 0 || pan > RP_PAN_MAX_DEG) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid pan");
        return ESP_FAIL;
    }
    bool ok = control_actuator((uint16_t)pan, RP_PUSH_STOP);
    char json[64];
    snprintf(json, sizeof(json), "{\"ok\":%s,\"pan\":%d}",
             ok ? "true" : "false", pan);
    return send_json(req, json);
}

static esp_err_t handle_pan_calibrate(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    bool ok = control_calibrate_pan();
    return send_json(req, ok ?
                     "{\"ok\":true,\"pan\":270}" :
                     "{\"ok\":false,\"error\":\"C30D did not acknowledge\"}");
}

static esp_err_t handle_wifi_status(httpd_req_t *req)
{
    td_wifi_status_t status;
    cJSON *root;
    char *json;
    esp_err_t err;

    wifi_mgr_get_status(&status);
    root = cJSON_CreateObject();
    if (root == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "station_connected",
                          status.station_connected);
    cJSON_AddBoolToObject(root, "ap_active",
                          status.access_point_active);
    cJSON_AddBoolToObject(root, "setup_ap",
                          status.setup_access_point);
    cJSON_AddBoolToObject(root, "ap_client",
                          status.access_point_client_connected);
    cJSON_AddStringToObject(root, "mode",
                            status.preferred_mode ==
                            TD_WIFI_MODE_ACCESS_POINT ?
                            "ap" : "station");
    cJSON_AddStringToObject(root, "ip", status.ip);
    cJSON_AddStringToObject(root, "url", status.url);
    cJSON_AddStringToObject(root, "saved_ssid", status.ssid);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    err = send_json(req, json);
    cJSON_free(json);
    return err;
}

static esp_err_t handle_wifi_scan(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    wifi_ap_record_t *records = heap_caps_calloc(
        20u, sizeof(*records), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint16_t count = 0u;
    esp_err_t err;
    cJSON *root;
    cJSON *items;
    char *json;

    if (records == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    err = wifi_mgr_scan(records, 20u, &count);
    root = cJSON_CreateObject();
    items = cJSON_AddArrayToObject(root, "networks");
    if (root == NULL || items == NULL) {
        heap_caps_free(records);
        cJSON_Delete(root);
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "ok", err == ESP_OK);
    if (err != ESP_OK) {
        cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
    }
    for (uint16_t i = 0; i < count; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "ssid",
                                (const char *)records[i].ssid);
        cJSON_AddNumberToObject(item, "rssi", records[i].rssi);
        cJSON_AddNumberToObject(item, "auth", records[i].authmode);
        cJSON_AddItemToArray(items, item);
    }
    heap_caps_free(records);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    err = send_json(req, json);
    cJSON_free(json);
    return err;
}

static esp_err_t handle_wifi_connect(httpd_req_t *req)
{
    char body[512];
    cJSON *root = NULL;
    cJSON *ssid;
    cJSON *password;
    cJSON *response;
    char *json;
    esp_err_t err;

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    ssid = cJSON_GetObjectItem(root, "ssid");
    password = cJSON_GetObjectItem(root, "password");
    if (!cJSON_IsString(ssid) || !cJSON_IsString(password)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "ssid/password required");
        return ESP_FAIL;
    }
    err = wifi_mgr_connect_station(ssid->valuestring,
                                   password->valuestring);
    cJSON_Delete(root);
    memset(body, 0, sizeof(body));
    response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", err == ESP_OK);
    if (err != ESP_OK) {
        cJSON_AddStringToObject(response, "error",
                                esp_err_to_name(err));
    }
    json = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    err = json == NULL ? ESP_ERR_NO_MEM : send_json(req, json);
    cJSON_free(json);
    return err;
}

static esp_err_t handle_wifi_mode(httpd_req_t *req)
{
    char body[256];
    cJSON *root = NULL;
    cJSON *mode;
    uint8_t selected;
    esp_err_t err;
    char json[128];

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    mode = cJSON_GetObjectItem(root, "mode");
    if (!cJSON_IsString(mode)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "mode required");
        return ESP_FAIL;
    }
    if (strcmp(mode->valuestring, "ap") == 0) {
        selected = TD_WIFI_MODE_ACCESS_POINT;
    } else if (strcmp(mode->valuestring, "station") == 0) {
        selected = TD_WIFI_MODE_STATION;
    } else {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "mode must be ap or station");
        return ESP_FAIL;
    }
    cJSON_Delete(root);
    err = wifi_mgr_set_mode(selected);
    snprintf(json, sizeof(json), "{\"ok\":%s,\"error\":\"%s\"}",
             err == ESP_OK ? "true" : "false", esp_err_to_name(err));
    return send_json(req, json);
}

static esp_err_t handle_wifi_forget(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    esp_err_t err = wifi_mgr_forget_station();
    char json[128];
    snprintf(json, sizeof(json), "{\"ok\":%s,\"error\":\"%s\"}",
             err == ESP_OK ? "true" : "false", esp_err_to_name(err));
    return send_json(req, json);
}

static esp_err_t handle_ai_status(httpd_req_t *req)
{
    td_agent_status_t status;
    char json[512];
    agent_service_snapshot(&status);
    snprintf(
        json, sizeof(json),
        "{\"configured\":%s,\"busy\":%s,\"done\":%s,"
        "\"requests\":%lu,\"model\":\"deepseek-flash\","
        "\"reasoning_effort\":\"max\"}",
        status.configured ? "true" : "false",
        status.busy ? "true" : "false",
        status.done ? "true" : "false",
        (unsigned long)status.requests);
    return send_json(req, json);
}

static esp_err_t handle_auth_check(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t handle_ai_config(httpd_req_t *req)
{
    char body[512];
    cJSON *root = NULL;
    cJSON *key;
    esp_err_t err;

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    key = cJSON_GetObjectItem(root, "api_key");
    if (!cJSON_IsString(key)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "api_key required");
        return ESP_FAIL;
    }
    err = agent_service_set_api_key(key->valuestring);
    cJSON_Delete(root);
    memset(body, 0, sizeof(body));
    return send_json(
        req, err == ESP_OK ?
        "{\"ok\":true,\"configured\":true}" :
        "{\"ok\":false,\"error\":\"API key storage failed\"}");
}

static esp_err_t handle_agent_chat(httpd_req_t *req)
{
    char body[TD_AGENT_MESSAGE_CAP + 128u];
    cJSON *root = NULL;
    cJSON *message;
    esp_err_t err;

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    message = cJSON_GetObjectItem(root, "message");
    if (!cJSON_IsString(message)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "message required");
        return ESP_FAIL;
    }
    err = agent_service_submit(message->valuestring);
    cJSON_Delete(root);
    if (err == ESP_ERR_INVALID_STATE) {
        return send_json(
            req,
            "{\"ok\":false,\"error\":\"configure_api_key_or_wait\"}");
    }
    if (err != ESP_OK) {
        return send_json(req, "{\"ok\":false,\"error\":\"agent_busy\"}");
    }
    return send_json(req, "{\"ok\":true,\"state\":\"thinking\"}");
}

static esp_err_t handle_agent_status(httpd_req_t *req)
{
    td_agent_status_t status;
    cJSON *root;
    char *json;
    esp_err_t err;

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    agent_service_snapshot(&status);
    root = cJSON_CreateObject();
    if (root == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddBoolToObject(root, "configured", status.configured);
    cJSON_AddBoolToObject(root, "busy", status.busy);
    cJSON_AddBoolToObject(root, "done", status.done);
    cJSON_AddNumberToObject(root, "sequence", status.sequence);
    cJSON_AddStringToObject(root, "reply", status.reply);
    cJSON_AddStringToObject(root, "error", status.error);
    cJSON_AddStringToObject(root, "tools", status.tool_summary);
    json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }
    err = send_json(req, json);
    cJSON_free(json);
    return err;
}

static esp_err_t handle_agent_state(httpd_req_t *req)
{
    char json[1280];
    agent_runtime_state_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t handle_agent_log(httpd_req_t *req)
{
    static char json[2048];
    rb_log_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t handle_bus_graph(httpd_req_t *req)
{
    static char json[2560];
    rb_graph_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t handle_params_get(httpd_req_t *req)
{
    static char json[1024];
    robot_params_json(json, sizeof(json));
    return send_json(req, json);
}

static esp_err_t handle_agent_mode(httpd_req_t *req)
{
    char body[256];
    cJSON *root = NULL;
    cJSON *on;
    cJSON *seconds;
    char json[160];
    uint32_t lease_seconds;

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    on = cJSON_GetObjectItem(root, "takeover");
    seconds = cJSON_GetObjectItem(root, "seconds");
    if (seconds != NULL && (!cJSON_IsNumber(seconds) ||
        seconds->valuedouble != seconds->valueint || seconds->valueint <= 0)) {
        cJSON_Delete(root);
        return send_json(req, "{\"ok\":false,\"error\":\"invalid lease seconds\"}");
    }
    lease_seconds = seconds == NULL ? 60u : (uint32_t)seconds->valueint;
    bool enabled = cJSON_IsBool(on) ? cJSON_IsTrue(on) : false;
    bool ok = agent_runtime_set_takeover(enabled, lease_seconds);
    cJSON_Delete(root);
    snprintf(json, sizeof(json),
             "{\"ok\":%s,\"takeover\":%s,\"lease_ms\":%u}",
             ok ? "true" : "false", enabled ? "true" : "false",
             (unsigned)agent_runtime_lease_remaining_ms());
    return send_json(req, json);
}

static esp_err_t handle_agent_lease(httpd_req_t *req)
{
    char body[128];
    cJSON *root = NULL;
    cJSON *seconds;
    uint32_t lease_seconds = 30u;
    char json[128];

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    seconds = cJSON_GetObjectItem(root, "seconds");
    if (seconds != NULL) {
        if (!cJSON_IsNumber(seconds) || seconds->valuedouble != seconds->valueint ||
            seconds->valueint <= 0) {
            cJSON_Delete(root);
            return send_json(req, "{\"ok\":false,\"error\":\"invalid lease seconds\"}");
        }
        lease_seconds = (uint32_t)seconds->valueint;
    }
    cJSON_Delete(root);
    if (!agent_runtime_lease(lease_seconds)) {
        return send_json(req, "{\"ok\":false,\"error\":\"not in takeover\"}");
    }
    snprintf(json, sizeof(json), "{\"ok\":true,\"lease_ms\":%u}",
             (unsigned)agent_runtime_lease_remaining_ms());
    return send_json(req, json);
}

static esp_err_t handle_agent_plan(httpd_req_t *req)
{
    static char body[2048];
    cJSON *root = NULL;
    char error[64] = {0};
    uint32_t plan_id = 0u;
    char json[192];

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    cJSON_Delete(root);
    if (!agent_runtime_submit_plan(body, error, sizeof(error), &plan_id)) {
        snprintf(json, sizeof(json),
                 "{\"ok\":false,\"error\":\"plan rejected\",\"detail\":\"%s\"}",
                 error);
        return send_json(req, json);
    }
    snprintf(json, sizeof(json), "{\"ok\":true,\"plan_id\":%u}",
             (unsigned)plan_id);
    return send_json(req, json);
}

static esp_err_t handle_agent_plan_cancel(httpd_req_t *req)
{
    bool ok;
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    ok = agent_runtime_cancel_plan(0u);
    return send_json(req, ok ? "{\"ok\":true}" :
                               "{\"ok\":false,\"error\":\"no plan\"}");
}

static esp_err_t handle_param_set(httpd_req_t *req)
{
    char body[256];
    cJSON *root = NULL;
    cJSON *name;
    cJSON *value;
    char json[160];

    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    if (read_json_body(req, body, sizeof(body), &root) != ESP_OK) {
        return ESP_FAIL;
    }
    name = cJSON_GetObjectItem(root, "name");
    value = cJSON_GetObjectItem(root, "value");
    if (!cJSON_IsString(name) || !cJSON_IsNumber(value)) {
        cJSON_Delete(root);
        return send_json(req, "{\"ok\":false,\"error\":\"name and value required\"}");
    }
    if (robot_param_id_from_name(name->valuestring) < 0) {
        cJSON_Delete(root);
        return send_json(req, "{\"ok\":false,\"error\":\"unknown parameter\"}");
    }
    bool ok = robot_param_set_by_name(name->valuestring, value->valuedouble);
    snprintf(json, sizeof(json), "{\"ok\":%s,\"name\":\"%s\"}",
             ok ? "true" : "false", name->valuestring);
    cJSON_Delete(root);
    return send_json(req, json);
}

static esp_err_t handle_face_enroll(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    esp_err_t err = face_service_request_enroll();
    if (err == ESP_OK) {
        voice_play_async("face_found");
    }
    char json[96];
    snprintf(json, sizeof(json), "{\"ok\":%s,\"accepted\":%s}",
             err == ESP_OK ? "true" : "false",
             err == ESP_OK ? "true" : "false");
    return send_json(req, json);
}

static esp_err_t handle_face_delete(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    return send_json(req,
                     face_service_delete_last() == ESP_OK ?
                     "{\"ok\":true,\"accepted\":true}" :
                     "{\"ok\":false,\"accepted\":false}");
}

static esp_err_t handle_face_clear(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_FAIL;
    }
    return send_json(req,
                     face_service_clear_all() == ESP_OK ?
                     "{\"ok\":true,\"accepted\":true}" :
                     "{\"ok\":false,\"accepted\":false}");
}

/* Read-only sample. The collector never starts the vehicle or writes flash. */
static esp_err_t handle_line_sample(httpd_req_t *req)
{
    if (!require_device_auth(req)) {
        return ESP_OK;
    }
    td_status_t status;
    app_state_snapshot(&status);
    const td_line_sample_t *sample = &status.line_learning;
    bool valid = sample->valid && status.stm32_online && status.mode == TD_MODE_LINE &&
                 app_rx_age_ms(app_now_us(), sample->frame_us) <= 180u;
    char json[360];
    int used = snprintf(json, sizeof(json),
        "{\"v\":1,\"valid\":%s,\"frame_us\":%lld,\"ccd_us\":%lld,"
        "\"ccd\":%u,\"confidence\":%u,\"speed\":%d,\"model\":%d,\"prediction\":%d,\"dark\":[",
        valid ? "true" : "false", (long long)sample->frame_us,
        (long long)sample->ccd_us, sample->ccd_center, sample->ccd_confidence,
        sample->speed_mm_s, TD_LINE_MODEL_AVAILABLE, sample->prediction);
    for (unsigned i = 0; i < TD_LINE_IMAGE_FEATURES; ++i) {
        used += snprintf(json + used, sizeof(json) - used, "%s%u",
                         i == 0 ? "" : ",", sample->dark[i]);
    }
    memcpy(json + used, "]}", 3);
    return send_json(req, json);
}

static esp_err_t handle_frame(httpd_req_t *req)
{
    uint8_t *bmp = NULL;
    size_t length = 0;
    esp_err_t err = camera_app_get_bmp(&bmp, &length);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "camera not ready");
    }
    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t send_err = httpd_resp_send(req, (const char *)bmp,
                                         (ssize_t)length);
    camera_app_free_bmp(bmp);
    return send_err;
}

static esp_err_t handle_c30d(httpd_req_t *req)
{
    const size_t json_capacity = TD_C30D_JSON_CAPACITY;
    td_status_t status;
    td_c30d_meta_t meta;
    td_c30d_sample_t samples[TD_C30D_HISTORY_CAPACITY];
    size_t sample_count;
    char *json = heap_caps_malloc(
        json_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    app_state_snapshot(&status);
    app_state_build_c30d_meta(&status, &meta);
    app_state_lock();
    sample_count = td_c30d_monitor_snapshot(
        samples, TD_C30D_HISTORY_CAPACITY);
    app_state_unlock();

    if (td_c30d_monitor_export_json(
            &meta, samples, sample_count, json, json_capacity) == 0u) {
        heap_caps_free(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    esp_err_t err = send_json(req, json);
    heap_caps_free(json);
    return err;
}

static esp_err_t handle_selftest(httpd_req_t *req)
{
    startup_report_t report;
    startup_selftest_snapshot(&report);
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) { httpd_resp_send_500(req); return ESP_ERR_NO_MEM; }
    cJSON_AddBoolToObject(root, "running", report.running);
    cJSON_AddBoolToObject(root, "ready", report.ready);
    cJSON_AddBoolToObject(root, "complete", report.complete);
    cJSON_AddBoolToObject(root, "announcing", report.announcing);
    cJSON_AddNumberToObject(root, "checked_at_ms", report.checked_at_ms);
    cJSON_AddNumberToObject(root, "errors", report.checks.errors);
    cJSON_AddNumberToObject(root, "attention", report.checks.attention);
    cJSON_AddStringToObject(root, "audio_error", esp_err_to_name(report.audio_error));
    cJSON *items = cJSON_AddArrayToObject(root, "items");
    if (items == NULL) { cJSON_Delete(root); httpd_resp_send_500(req); return ESP_ERR_NO_MEM; }
    for (int m = 0; report.ready && m < ST_ITEM_COUNT; ++m) {
        cJSON *item = cJSON_CreateObject();
        if (item == NULL) { cJSON_Delete(root); httpd_resp_send_500(req); return ESP_ERR_NO_MEM; }
        st_result_t result = report.checks.results[m];
        cJSON_AddStringToObject(item, "id", st_module_id(m));
        cJSON_AddStringToObject(item, "name", st_module_name(m));
        cJSON_AddStringToObject(item, "state", st_result_id(result));
        cJSON_AddStringToObject(item, "message", st_result_name(result));
        cJSON_AddStringToObject(item, "level", st_is_error(result) ? "error" :
            st_needs_attention(result) ? "attention" : "info");
        cJSON_AddItemToArray(items, item);
    }
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) { httpd_resp_send_500(req); return ESP_ERR_NO_MEM; }
    esp_err_t err = send_json(req, json);
    cJSON_free(json);
    return err;
}

static esp_err_t handle_c30d_csv(httpd_req_t *req)
{
    const size_t csv_capacity = 16384u;
    char *csv = heap_caps_malloc(
        csv_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (csv == NULL) {
        httpd_resp_send_500(req);
        return ESP_ERR_NO_MEM;
    }

    app_state_lock();
    size_t length = td_c30d_monitor_export_csv(csv, csv_capacity);
    app_state_unlock();

    httpd_resp_set_type(req, "text/csv; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(
        req, "Content-Disposition",
        "attachment; filename=c30d_recent.csv");
    esp_err_t err;
    if (length == 0u) {
        err = httpd_resp_sendstr(
            req,
            "time_ms,online,state,faults,left_mm_s,right_mm_s,"
            "yaw_cdeg,distance_cm,battery_mv\r\n");
    } else {
        err = httpd_resp_send(req, csv, (ssize_t)length);
    }
    heap_caps_free(csv);
    return err;
}

esp_err_t web_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;
    config.core_id = 0;
    config.task_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    config.max_uri_handlers = 48;
    config.lru_purge_enable = true;
    config.max_open_sockets = 5;

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "http start");

    const httpd_uri_t handlers[] = {
        {.uri = "/", .method = HTTP_GET, .handler = handle_root},
        {.uri = "/api/status", .method = HTTP_GET, .handler = handle_status},
        {.uri = "/api/selftest", .method = HTTP_GET, .handler = handle_selftest},
        {.uri = "/api/say", .method = HTTP_POST, .handler = handle_say},
        {.uri = "/api/start", .method = HTTP_POST, .handler = handle_start},
        {.uri = "/api/stop", .method = HTTP_POST, .handler = handle_stop},
        {.uri = "/api/estop", .method = HTTP_POST, .handler = handle_estop},
        {.uri = "/api/clear", .method = HTTP_POST, .handler = handle_clear},
        {.uri = "/api/inspect", .method = HTTP_POST,
         .handler = handle_inspect},
        {.uri = "/api/move", .method = HTTP_POST, .handler = handle_move},
        {.uri = "/api/actuator", .method = HTTP_POST,
         .handler = handle_actuator},
        {.uri = "/api/actuator/calibrate", .method = HTTP_POST,
         .handler = handle_pan_calibrate},
        {.uri = "/api/tune", .method = HTTP_POST, .handler = handle_tune},
        {.uri = "/api/wifi/status", .method = HTTP_GET,
         .handler = handle_wifi_status},
        {.uri = "/api/wifi/scan", .method = HTTP_GET,
         .handler = handle_wifi_scan},
        {.uri = "/api/wifi/connect", .method = HTTP_POST,
         .handler = handle_wifi_connect},
        {.uri = "/api/wifi/mode", .method = HTTP_POST,
         .handler = handle_wifi_mode},
        {.uri = "/api/wifi/forget", .method = HTTP_POST,
         .handler = handle_wifi_forget},
        {.uri = "/api/ai/status", .method = HTTP_GET,
         .handler = handle_ai_status},
        {.uri = "/api/auth/check", .method = HTTP_GET,
         .handler = handle_auth_check},
        {.uri = "/api/ai/config", .method = HTTP_POST,
         .handler = handle_ai_config},
        {.uri = "/api/agent/chat", .method = HTTP_POST,
         .handler = handle_agent_chat},
        {.uri = "/api/agent/status", .method = HTTP_GET,
         .handler = handle_agent_status},
        {.uri = "/api/agent/state", .method = HTTP_GET,
         .handler = handle_agent_state},
        {.uri = "/api/agent/log", .method = HTTP_GET,
         .handler = handle_agent_log},
        {.uri = "/api/agent/mode", .method = HTTP_POST,
         .handler = handle_agent_mode},
        {.uri = "/api/agent/lease", .method = HTTP_POST,
         .handler = handle_agent_lease},
        {.uri = "/api/agent/plan", .method = HTTP_POST,
         .handler = handle_agent_plan},
        {.uri = "/api/agent/plan/cancel", .method = HTTP_POST,
         .handler = handle_agent_plan_cancel},
        {.uri = "/api/bus/graph", .method = HTTP_GET,
         .handler = handle_bus_graph},
        {.uri = "/api/params", .method = HTTP_GET,
         .handler = handle_params_get},
        {.uri = "/api/params/set", .method = HTTP_POST,
         .handler = handle_param_set},
        {.uri = "/api/c30d", .method = HTTP_GET, .handler = handle_c30d},
        {.uri = "/api/c30d.csv", .method = HTTP_GET,
         .handler = handle_c30d_csv},
        {.uri = "/api/face/enroll", .method = HTTP_POST,
         .handler = handle_face_enroll},
        {.uri = "/api/face/delete", .method = HTTP_POST,
         .handler = handle_face_delete},
        {.uri = "/api/face/clear", .method = HTTP_POST,
         .handler = handle_face_clear},
        {.uri = "/api/line-sample", .method = HTTP_GET, .handler = handle_line_sample},
        {.uri = "/frame.bmp", .method = HTTP_GET, .handler = handle_frame},
    };
    for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); ++i) {
        ESP_RETURN_ON_ERROR(
            httpd_register_uri_handler(server, &handlers[i]),
            TAG, "register HTTP handler");
    }
    app_state_lock();
    app_state_mutable()->web_ready = true;
    app_state_unlock();
    ESP_LOGI(TAG, "web console ready at %s/", TD_AP_URL);
    return ESP_OK;
}
