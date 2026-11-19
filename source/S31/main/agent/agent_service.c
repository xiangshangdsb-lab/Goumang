#include "agent_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_config.h"
#include "app/app_state.h"
#include "agent_policy.h"
#include "agent_runtime.h"
#include "perception/camera_app.h"
#include "cJSON.h"
#include "control/control.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "perception/face_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "nvs.h"
#include "perception/object_service.h"
#include "bus/robot_bus.h"
#include "protocol/robot_protocol.h"

#define AGENT_NVS_NAMESPACE       "td_ai"
#define AGENT_NVS_KEY_API         "api_key"
#define AGENT_MAX_MESSAGE         TD_AGENT_MESSAGE_CAP
#define AGENT_MAX_ROUNDS          4
#define AGENT_MAX_HTTP_BODY       (192u * 1024u)
#define AGENT_MODEL               "deepseek-flash"
#define AGENT_URL                 "https://api.deepseek.com/chat/completions"

typedef struct {
    char text[AGENT_MAX_MESSAGE];
} agent_request_t;

typedef struct {
    char *data;
    size_t length;
    size_t capacity;
    bool overflow;
} http_response_t;

static SemaphoreHandle_t s_status_mutex;
static QueueHandle_t s_request_queue;
static td_agent_status_t s_status;
static char s_api_key[256];

static void set_error(const char *message)
{
    if (s_status_mutex != NULL) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        strlcpy(s_status.error, message, sizeof(s_status.error));
        s_status.reply[0] = '\0';
        xSemaphoreGive(s_status_mutex);
    }
}

static bool request_has_error(void)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    bool failed = s_status.error[0] != '\0';
    xSemaphoreGive(s_status_mutex);
    return failed;
}

static esp_err_t load_api_key(void)
{
    nvs_handle_t handle;
    size_t length = sizeof(s_api_key);
    esp_err_t err = nvs_open(AGENT_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_api_key[0] = '\0';
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_get_str(handle, AGENT_NVS_KEY_API, s_api_key, &length);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_api_key[0] = '\0';
        return ESP_OK;
    }
    return err;
}

static void add_frame_age(cJSON *object, const char *key, int64_t now_us, int64_t rx_us)
{
    uint32_t age_ms = app_rx_age_ms(now_us, rx_us);
    if (age_ms == UINT32_MAX) {
        cJSON_AddNullToObject(object, key);
    } else {
        cJSON_AddNumberToObject(object, key, age_ms);
    }
}

static cJSON *build_status_json(void)
{
    td_status_t status;
    cJSON *root = cJSON_CreateObject();
    cJSON *line;
    cJSON *imu;
    cJSON *wifi;
    cJSON *vision;
    td_face_status_t face;
    td_object_status_t objects;

    if (root == NULL) {
        return NULL;
    }
    app_state_snapshot(&status);
    face_service_snapshot(&face);
    object_service_snapshot(&objects);
    cJSON_AddStringToObject(root, "mode", td_mode_name(status.mode));
    cJSON_AddBoolToObject(root, "c30d_online", status.stm32_online);
    cJSON_AddNumberToObject(root, "c30d_fresh_limit_ms", TD_STM32_ONLINE_MS);
    cJSON *ages = cJSON_AddObjectToObject(root, "c30d_frame_age_ms");
    int64_t now_us = app_now_us();
    add_frame_age(ages, "chassis", now_us, status.stm32_chassis_rx_us);
    add_frame_age(ages, "sensor", now_us, status.stm32_sensor_rx_us);
    add_frame_age(ages, "safety", now_us, status.stm32_safety_rx_us);
    add_frame_age(ages, "line", now_us, status.stm32_line_rx_us);
    add_frame_age(ages, "imu", now_us, status.stm32_imu_rx_us);
    add_frame_age(ages, "motor", now_us, status.stm32_motor_rx_us);
    cJSON_AddNumberToObject(root, "robot_state", status.safety.robot_state);
    cJSON_AddNumberToObject(root, "fault_flags", status.safety.fault_flags);
    cJSON_AddNumberToObject(root, "battery_mv", status.sensor.battery_mv);
    cJSON_AddNumberToObject(root, "ultrasonic_cm",
                            status.sensor.distance_cm);
    cJSON_AddNumberToObject(root, "pan_deg", status.pan_deg);
    cJSON_AddNumberToObject(root, "left_mm_s",
                            status.chassis.left_speed_mm_s);
    cJSON_AddNumberToObject(root, "right_mm_s",
                            status.chassis.right_speed_mm_s);
    cJSON_AddNumberToObject(root, "yaw_cdeg", status.chassis.yaw_cdeg);
    cJSON_AddNumberToObject(root, "output_left", status.out_left);
    cJSON_AddNumberToObject(root, "output_right", status.out_right);
    cJSON_AddNumberToObject(root, "encoder_delta_a",
                            status.motor.encoder_delta_a);
    cJSON_AddNumberToObject(root, "encoder_delta_b",
                            status.motor.encoder_delta_b);
    cJSON_AddNumberToObject(root, "pwm_a", status.motor.pwm_a);
    cJSON_AddNumberToObject(root, "pwm_b", status.motor.pwm_b);
    cJSON_AddNumberToObject(root, "target_a_mm_s",
                            status.motor.target_a_mm_s);
    cJSON_AddNumberToObject(root, "target_b_mm_s",
                            status.motor.target_b_mm_s);
    cJSON_AddNumberToObject(root, "reported_pan_deg",
                            status.motor.pan_deg);
    cJSON_AddBoolToObject(root, "pan_known", (status.motor.pan_flags & RP_PAN_KNOWN) != 0u);
    cJSON_AddBoolToObject(root, "pan_moving", (status.motor.pan_flags & RP_PAN_MOVING) != 0u);
    cJSON_AddBoolToObject(root, "pan_boot_return", (status.motor.pan_flags & RP_PAN_BOOT_RETURN) != 0u);
    cJSON_AddNumberToObject(root, "control_source",
                            status.motor.control_source);

    line = cJSON_AddObjectToObject(root, "line");
    cJSON_AddNumberToObject(line, "flags", status.line_state.flags);
    cJSON_AddNumberToObject(line, "source", status.line_state.source);
    cJSON_AddNumberToObject(line, "ccd_center",
                            status.line_state.ccd_center);
    cJSON_AddNumberToObject(line, "ccd_confidence",
                            status.line_state.ccd_confidence);
    cJSON_AddNumberToObject(line, "error", status.line_state.line_error);
    cJSON_AddNumberToObject(line, "lf_bits", status.line_state.lf_bits);
    cJSON_AddNumberToObject(line, "left_command",
                            status.line_state.left_command);
    cJSON_AddNumberToObject(line, "right_command",
                            status.line_state.right_command);
    cJSON_AddNumberToObject(line, "rewind_progress",
                            status.line_state.rewind_progress);

    imu = cJSON_AddObjectToObject(root, "imu");
    cJSON_AddNumberToObject(imu, "gyro_x_ddps", status.imu.gyro_x_ddps);
    cJSON_AddNumberToObject(imu, "gyro_y_ddps", status.imu.gyro_y_ddps);
    cJSON_AddNumberToObject(imu, "gyro_z_ddps", status.imu.gyro_z_ddps);
    cJSON_AddNumberToObject(imu, "accel_x_mg", status.imu.accel_x_mg);
    cJSON_AddNumberToObject(imu, "accel_y_mg", status.imu.accel_y_mg);
    cJSON_AddNumberToObject(imu, "accel_z_mg", status.imu.accel_z_mg);

    wifi = cJSON_AddObjectToObject(root, "network");
    cJSON_AddBoolToObject(wifi, "station_connected", status.sta_connected);
    cJSON_AddBoolToObject(wifi, "access_point", status.ap_started);
    cJSON_AddStringToObject(wifi, "url", status.url);

    vision = cJSON_AddObjectToObject(root, "vision");
    cJSON_AddBoolToObject(vision, "camera_ready", status.camera_ready);
    cJSON_AddBoolToObject(vision, "line_found", status.line.line_found);
    cJSON_AddNumberToObject(vision, "line_error", status.line.error);
    cJSON_AddNumberToObject(vision, "line_confidence",
                            status.line.confidence);
    vision = cJSON_AddArrayToObject(root, "objects");
    for (uint8_t i = 0u; i < objects.count && i < TD_MAX_OBJECTS; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", objects.items[i].name);
        cJSON_AddNumberToObject(item, "confidence",
                                objects.items[i].confidence);
        cJSON_AddItemToArray(vision, item);
    }
    {
        cJSON *face_json = cJSON_AddObjectToObject(root, "face");
        cJSON_AddBoolToObject(face_json, "ready", face.ready);
        cJSON_AddBoolToObject(face_json, "found", face.face_found);
        cJSON_AddBoolToObject(face_json, "recognized", face.recognized);
        cJSON_AddNumberToObject(face_json, "id", face.face_id);
        cJSON_AddNumberToObject(face_json, "registered",
                                face.registered_count);
    }
    return root;
}

static void add_tool(cJSON *tools,
                     const char *name,
                     const char *description,
                     cJSON *properties,
                     cJSON *required)
{
    cJSON *tool = cJSON_CreateObject();
    cJSON *function = cJSON_CreateObject();
    cJSON *parameters = cJSON_CreateObject();

    if (tool == NULL || function == NULL || parameters == NULL) {
        cJSON_Delete(tool);
        cJSON_Delete(function);
        cJSON_Delete(parameters);
        cJSON_Delete(properties);
        cJSON_Delete(required);
        return;
    }
    cJSON_AddStringToObject(tool, "type", "function");
    cJSON_AddStringToObject(function, "name", name);
    cJSON_AddStringToObject(function, "description", description);
    cJSON_AddStringToObject(parameters, "type", "object");
    if (properties != NULL) {
        cJSON_AddItemToObject(parameters, "properties", properties);
    }
    if (required != NULL) {
        cJSON_AddItemToObject(parameters, "required", required);
    }
    cJSON_AddItemToObject(function, "parameters", parameters);
    cJSON_AddItemToObject(tool, "function", function);
    cJSON_AddItemToArray(tools, tool);
}

static cJSON *number_property(const char *description,
                              int minimum,
                              int maximum)
{
    cJSON *property = cJSON_CreateObject();
    cJSON_AddStringToObject(property, "type", "integer");
    cJSON_AddStringToObject(property, "description", description);
    cJSON_AddNumberToObject(property, "minimum", minimum);
    cJSON_AddNumberToObject(property, "maximum", maximum);
    return property;
}

static cJSON *string_property(const char *description)
{
    cJSON *property = cJSON_CreateObject();
    cJSON_AddStringToObject(property, "type", "string");
    cJSON_AddStringToObject(property, "description", description);
    return property;
}

static cJSON *string_array(const char *value)
{
    cJSON *array = cJSON_CreateArray();
    cJSON_AddItemToArray(array, cJSON_CreateString(value));
    return array;
}

static cJSON *build_tools(void)
{
    cJSON *tools = cJSON_CreateArray();
    if (tools == NULL) {
        return NULL;
    }
    add_tool(tools, "get_robot_status", "读取实时状态与遥测。", NULL, NULL);
    add_tool(tools, "get_telemetry", "读取 C30D、传感器、网络及视觉状态。", NULL, NULL);
    add_tool(tools, "get_line_state", "读取本地巡线状态及障碍标志。", NULL, NULL);
    add_tool(tools, "get_vision_summary", "读取本地通用目标检测摘要。", NULL, NULL);
    add_tool(tools, "stop_robot", "立即停止底盘。", NULL, NULL);
    add_tool(tools, "emergency_stop", "触发急停。", NULL, NULL);
    add_tool(tools, "capture_photo",
             "停车且云台稳定时拍摄并返回新照片供视觉分析。", NULL, NULL);
    add_tool(tools, "get_takeover_status", "读取接管模式和租约。", NULL, NULL);

    cJSON *properties = cJSON_CreateObject();
    cJSON_AddItemToObject(properties, "angle_deg",
                         number_property("云台角度，270 度为正前方", 0, RP_PAN_MAX_DEG));
    add_tool(tools, "set_pan", "停车时调整观察角度，等云台停稳后拍照。",
             properties, string_array("angle_deg"));

    properties = cJSON_CreateObject();
    cJSON_AddItemToObject(properties, "mode",
                         string_property("PATROL, INSPECT, DIAGNOSE, WAIT"));
    cJSON_AddItemToObject(properties, "action",
                         string_property("FOLLOW_LINE, FORWARD_SHORT, "
                                         "TURN_LEFT_SMALL, TURN_RIGHT_SMALL, STOP"));
    cJSON_AddItemToObject(properties, "category",
                         string_property("FUNGAL, OOMYCETE, BACTERIAL, VIRAL, "
                                         "NEMATODE, PEST, ABIOTIC, HEALTHY, UNKNOWN"));
    cJSON_AddItemToObject(properties, "observation",
                         string_property("可见证据；无照片时用 UNKNOWN 并留空"));
    cJSON *boolean = cJSON_CreateObject();
    cJSON_AddStringToObject(boolean, "type", "boolean");
    cJSON_AddItemToObject(properties, "needs_more_images", boolean);
    boolean = cJSON_CreateObject();
    cJSON_AddStringToObject(boolean, "type", "boolean");
    cJSON_AddItemToObject(properties, "spray_authorized", boolean);
    cJSON *required = string_array("mode");
    const char *fields[] = {"action", "category", "observation",
                            "needs_more_images", "spray_authorized"};
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        cJSON_AddItemToArray(required, cJSON_CreateString(fields[i]));
    }
    add_tool(tools, "field_decision", "提交一个巡检建议。设备决定是否执行及速度和时长。"
             "喷药授权必须为 false。", properties, required);
    return tools;
}

static char *status_text(void)
{
    cJSON *status = build_status_json();
    char *text;
    if (status == NULL) {
        return NULL;
    }
    text = cJSON_PrintUnformatted(status);
    cJSON_Delete(status);
    return text;
}

static cJSON *build_messages(const char *user_message)
{
    cJSON *messages = cJSON_CreateArray();
    cJSON *system = cJSON_CreateObject();
    cJSON *user = cJSON_CreateObject();
    char *status_json = status_text();
    const char *telemetry = status_json != NULL ? status_json : "{}";
    size_t user_capacity = strlen(telemetry) + strlen(user_message) + 512u;
    char *user_buffer = heap_caps_malloc(user_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    agent_state_t agent;

    if (messages == NULL || system == NULL || user == NULL || user_buffer == NULL) {
        cJSON_Delete(messages);
        cJSON_Delete(system);
        cJSON_Delete(user);
        cJSON_free(status_json);
        heap_caps_free(user_buffer);
        return NULL;
    }
    agent_runtime_state_snapshot(&agent);
    int written = snprintf(
        user_buffer, user_capacity,
        "Telemetry: %s\nMode: %s (lease %u s, plan %s)\n"
        "User request: %s\n\n"
        "Treat C30D telemetry as stale if its group age is null or at least "
        "c30d_fresh_limit_ms. Read telemetry again before deciding to move. "
        "Call a tool before claiming a physical action succeeded.",
        telemetry,
        agent.mode == AGENT_MODE_TAKEOVER ? "TAKEOVER" : "ASSIST",
        (unsigned)(agent.lease_remaining_ms / 1000u),
        agent.plan_running ? "running" : "idle",
        user_message);
    cJSON_free(status_json);
    if (written < 0 || (size_t)written >= user_capacity) {
        heap_caps_free(user_buffer);
        cJSON_Delete(messages);
        cJSON_Delete(system);
        cJSON_Delete(user);
        return NULL;
    }

    cJSON_AddStringToObject(system, "role", "system");
    cJSON_AddStringToObject(
        system, "content",
        "你是句芒履带机器人的巡检助手，使用简洁中文。移动前读遥测并拍摄新照片。"
        "优先建议本地巡线，单次只提交一种动作。短距离移动参数由 S31 决定。"
        "发现植株异常、障碍、状态过期或需要补拍时先停车。底盘和云台停稳后调用 capture_photo。"
        "观察不足时分类 UNKNOWN，可以请求补拍叶片背面或其他角度。"
        "仅报告图像可见证据及可能分类，区分真菌、卵菌和非侵染性异常。"
        "spray_authorized 必须为 false。施药由人工根据登记用法决定。"
        "你可以请求停车或急停。解除急停和接管授权由操作员完成。"
        "动作排队后说明已提交，收到设备状态后才能确认完成。");
    cJSON_AddStringToObject(user, "role", "user");
    cJSON_AddStringToObject(user, "content", user_buffer);
    heap_caps_free(user_buffer);
    cJSON_AddItemToArray(messages, system);
    cJSON_AddItemToArray(messages, user);
    return messages;
}

static cJSON *build_request(cJSON *messages, cJSON *tools)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *thinking = cJSON_CreateObject();
    if (root == NULL || thinking == NULL) {
        cJSON_Delete(root);
        cJSON_Delete(thinking);
        return NULL;
    }
    cJSON_AddStringToObject(root, "model", AGENT_MODEL);
    cJSON_AddBoolToObject(root, "stream", false);
    cJSON_AddNumberToObject(root, "max_tokens", 4096);
    cJSON_AddStringToObject(root, "reasoning_effort", "max");
    cJSON_AddStringToObject(thinking, "type", "enabled");
    cJSON_AddItemToObject(root, "thinking", thinking);
    cJSON_AddItemReferenceToObject(root, "messages", messages);
    cJSON_AddItemReferenceToObject(root, "tools", tools);
    return root;
}

static esp_err_t http_event_handler(esp_http_client_event_t *event)
{
    http_response_t *response = (http_response_t *)event->user_data;
    if (response == NULL) {
        return ESP_OK;
    }
    if (event->event_id == HTTP_EVENT_ON_DATA && event->data_len > 0) {
        size_t required = response->length + (size_t)event->data_len + 1u;
        if (required > AGENT_MAX_HTTP_BODY) {
            response->overflow = true;
            return ESP_OK;
        }
        if (required > response->capacity) {
            size_t capacity = response->capacity == 0u ? 8192u :
                              response->capacity;
            while (capacity < required) {
                capacity *= 2u;
            }
            if (capacity > AGENT_MAX_HTTP_BODY) {
                capacity = AGENT_MAX_HTTP_BODY;
            }
            char *grown = heap_caps_realloc(
                response->data, capacity,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (grown == NULL) {
                response->overflow = true;
                return ESP_OK;
            }
            response->data = grown;
            response->capacity = capacity;
        }
        memcpy(response->data + response->length, event->data,
               (size_t)event->data_len);
        response->length += (size_t)event->data_len;
        response->data[response->length] = '\0';
    }
    return ESP_OK;
}

static esp_err_t call_deepseek(cJSON *request, char **response_text,
                               int *http_status)
{
    char *body = cJSON_PrintUnformatted(request);
    char authorization[300];
    http_response_t response = {0};
    esp_http_client_config_t config = {
        .url = AGENT_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 90000,
        .buffer_size = 4096,
        .buffer_size_tx = 4096,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_event_handler,
        .user_data = &response,
    };
    esp_http_client_handle_t client;
    esp_err_t err;

    if (body == NULL) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    bool configured = s_status.configured;
    if (configured) {
        snprintf(authorization, sizeof(authorization), "Bearer %s", s_api_key);
    }
    xSemaphoreGive(s_status_mutex);
    if (!configured) {
        cJSON_free(body);
        return ESP_ERR_INVALID_STATE;
    }
    client = esp_http_client_init(&config);
    if (client == NULL) {
        cJSON_free(body);
        return ESP_ERR_NO_MEM;
    }
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Authorization", authorization);
    esp_http_client_set_post_field(client, body, (int)strlen(body));
    err = esp_http_client_perform(client);
    *http_status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    cJSON_free(body);
    if (response.overflow) {
        heap_caps_free(response.data);
        return ESP_ERR_INVALID_SIZE;
    }
    if (err != ESP_OK) {
        heap_caps_free(response.data);
        return err;
    }
    if (response.data == NULL) {
        response.data = heap_caps_calloc(1u, 1u, MALLOC_CAP_8BIT);
    }
    *response_text = response.data;
    return response.data == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

static cJSON *tool_result_ok(const char *action)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "action", action);
    return root;
}

static cJSON *tool_result_error(const char *message)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(root, "error", message);
    return root;
}

static cJSON *tool_result_command(const char *action, bool ok)
{
    return ok ? tool_result_ok(action) :
                tool_result_error("C30D did not acknowledge the command");
}

static cJSON *line_state_json(void)
{
    td_status_t status;
    cJSON *root = cJSON_CreateObject();

    if (root == NULL) {
        return NULL;
    }
    app_state_snapshot(&status);
    int64_t now_us = app_now_us();
    add_frame_age(root, "age_ms", now_us, status.stm32_line_rx_us);
    cJSON_AddBoolToObject(root, "fresh", status.stm32_online &&
        app_rx_age_ms(now_us, status.stm32_line_rx_us) < TD_STM32_ONLINE_MS);
    cJSON_AddNumberToObject(root, "flags", status.line_state.flags);
    cJSON_AddNumberToObject(root, "source", status.line_state.source);
    cJSON_AddNumberToObject(root, "ccd_center",
                            status.line_state.ccd_center);
    cJSON_AddNumberToObject(root, "ccd_confidence",
                            status.line_state.ccd_confidence);
    cJSON_AddNumberToObject(root, "error", status.line_state.line_error);
    cJSON_AddNumberToObject(root, "lf_bits", status.line_state.lf_bits);
    cJSON_AddNumberToObject(root, "left_command",
                            status.line_state.left_command);
    cJSON_AddNumberToObject(root, "right_command",
                            status.line_state.right_command);
    cJSON_AddNumberToObject(root, "rewind_progress",
                            status.line_state.rewind_progress);
    cJSON_AddNumberToObject(root, "distance_cm",
                            status.line_state.distance_cm);
    cJSON_AddNumberToObject(root, "pan_deg", status.pan_deg);
    cJSON_AddBoolToObject(root, "c30d_online", status.stm32_online);
    return root;
}

static cJSON *vision_summary_json(void)
{
    td_face_status_t face;
    td_object_status_t objects;
    cJSON *root = cJSON_CreateObject();
    cJSON *list;

    if (root == NULL) {
        return NULL;
    }
    face_service_snapshot(&face);
    object_service_snapshot(&objects);
    cJSON_AddBoolToObject(root, "face_ready", face.ready);
    cJSON_AddBoolToObject(root, "face_found", face.face_found);
    cJSON_AddBoolToObject(root, "face_recognized", face.recognized);
    cJSON_AddNumberToObject(root, "face_id", face.face_id);
    cJSON_AddNumberToObject(root, "registered_faces",
                            face.registered_count);
    list = cJSON_AddArrayToObject(root, "objects");
    for (uint8_t i = 0u; i < objects.count && i < TD_MAX_OBJECTS; ++i) {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "name", objects.items[i].name);
        cJSON_AddNumberToObject(item, "confidence",
                                objects.items[i].confidence);
        cJSON_AddItemToArray(list, item);
    }
    return root;
}

typedef struct {
    int64_t photo_us;
    bool motion_used;
} field_request_t;

static bool string_in(const cJSON *value, const char *const *options, size_t count)
{
    if (!cJSON_IsString(value)) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        if (strcmp(value->valuestring, options[i]) == 0) {
            return true;
        }
    }
    return false;
}

static cJSON *field_decision(const cJSON *args, field_request_t *field)
{
    static const char *const modes[] = {"PATROL", "INSPECT", "DIAGNOSE", "WAIT"};
    static const char *const categories[] = {
        "FUNGAL", "OOMYCETE", "BACTERIAL", "VIRAL", "NEMATODE", "PEST",
        "ABIOTIC", "HEALTHY", "UNKNOWN"
    };
    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(args, "mode");
    const cJSON *navigation = cJSON_GetObjectItemCaseSensitive(args, "action");
    const cJSON *category = cJSON_GetObjectItemCaseSensitive(args, "category");
    const cJSON *observation = cJSON_GetObjectItemCaseSensitive(args, "observation");
    const cJSON *needs_images = cJSON_GetObjectItemCaseSensitive(args, "needs_more_images");
    const cJSON *spray = cJSON_GetObjectItemCaseSensitive(args, "spray_authorized");
    field_action_t action = agent_policy_field_action(
        cJSON_IsString(navigation) ? navigation->valuestring : NULL);
    bool photo_fresh = field->photo_us > 0 && app_now_us() - field->photo_us <= 30000000;
    const char *blocked = NULL;

    if (!string_in(mode, modes, sizeof(modes) / sizeof(modes[0])) ||
        !string_in(category, categories, sizeof(categories) / sizeof(categories[0])) ||
        action == FIELD_INVALID || !cJSON_IsString(observation) ||
        strlen(observation->valuestring) > 360u ||
        !cJSON_IsBool(needs_images) || !cJSON_IsFalse(spray)) {
        blocked = "invalid field decision";
    } else if (!photo_fresh && strcmp(category->valuestring, "UNKNOWN") != 0) {
        blocked = "classification needs a fresh photo";
    } else if (action == FIELD_STOP) {
        if (!control_halt()) {
            blocked = "stop not acknowledged";
        }
    } else if (strcmp(mode->valuestring, "PATROL") != 0 ||
               cJSON_IsTrue(needs_images) || !photo_fresh) {
        blocked = "stop and capture a fresh image before continuing";
    } else if (field->motion_used) {
        blocked = "one movement per request";
    } else if (!agent_runtime_allow_control("field_decision")) {
        blocked = "control budget exhausted or takeover lease invalid";
    } else {
        field->motion_used = true;
        blocked = control_field_move(action);
        if (blocked == NULL) {
            field->photo_us = 0;
        }
    }
    if (blocked != NULL) {
        control_field_cancel();
        rb_log_event("field", "decision_blocked", blocked);
        return tool_result_error(blocked);
    }
    rb_log_event("field", navigation->valuestring, category->valuestring);
    cJSON *result = tool_result_ok(navigation->valuestring);
    cJSON_AddStringToObject(result, "category", category->valuestring);
    cJSON_AddBoolToObject(result, "spray_authorized", false);
    if (action != FIELD_STOP) {
        cJSON_AddStringToObject(result, "status", "scheduled");
    }
    return result;
}

static cJSON *capture_tool_content(field_request_t *field)
{
    uint8_t *jpeg = NULL;
    size_t bytes = 0;
    int64_t capture_after = app_now_us() + 180000;
    int64_t deadline = capture_after + 650000;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    field->photo_us = 0;

    while (app_now_us() < deadline && control_field_stationary()) {
        err = camera_app_get_jpeg(capture_after, &jpeg, &bytes);
        if (err != ESP_ERR_NOT_FOUND) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (err != ESP_OK || !control_field_stationary()) {
        heap_caps_free(jpeg);
        return NULL;
    }
    static const char prefix[] = "data:image/jpeg;base64,";
    size_t capacity = sizeof(prefix) + 4u * ((bytes + 2u) / 3u);
    char *url = heap_caps_malloc(capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    size_t used = 0u;
    int result = -1;
    if (url != NULL) {
        memcpy(url, prefix, sizeof(prefix) - 1u);
        result = mbedtls_base64_encode((unsigned char *)url + sizeof(prefix) - 1u,
                                      capacity - sizeof(prefix) + 1u, &used, jpeg, bytes);
    }
    heap_caps_free(jpeg);
    if (result != 0) {
        heap_caps_free(url);
        return NULL;
    }
    url[sizeof(prefix) - 1u + used] = '\0';

    cJSON *content = cJSON_CreateArray();
    cJSON *image = cJSON_CreateObject();
    cJSON *image_url = cJSON_CreateObject();
    if (content == NULL || image == NULL || image_url == NULL ||
        cJSON_AddStringToObject(image_url, "url", url) == NULL) {
        cJSON_Delete(content);
        cJSON_Delete(image);
        cJSON_Delete(image_url);
        heap_caps_free(url);
        return NULL;
    }
    heap_caps_free(url);
    cJSON_AddStringToObject(image, "type", "image_url");
    cJSON_AddItemToObject(image, "image_url", image_url);
    cJSON_AddItemToArray(content, image);
    field->photo_us = capture_after;
    rb_log_event("field", "photo_captured", "fresh JPEG");
    return content;
}

static cJSON *execute_tool(const char *name, const char *arguments_json,
                           field_request_t *field)
{
    cJSON *arguments = cJSON_ParseWithOpts(arguments_json, NULL, true);
    cJSON *result;

    if (strcmp(name, "get_robot_status") == 0 || strcmp(name, "get_telemetry") == 0) {
        result = build_status_json();
    } else if (strcmp(name, "get_line_state") == 0) {
        result = line_state_json();
    } else if (strcmp(name, "get_vision_summary") == 0) {
        result = vision_summary_json();
    } else if (strcmp(name, "stop_robot") == 0) {
        result = tool_result_command(name, control_halt());
    } else if (strcmp(name, "emergency_stop") == 0) {
        result = tool_result_command(name, control_estop());
    } else if (strcmp(name, "field_decision") == 0) {
        result = field_decision(arguments, field);
    } else if (strcmp(name, "set_pan") == 0) {
        cJSON *angle = cJSON_GetObjectItemCaseSensitive(arguments, "angle_deg");
        bool valid = cJSON_IsNumber(angle) && angle->valuedouble == angle->valueint &&
                     agent_policy_pan_angle(angle->valueint) &&
                     control_field_stationary();
        field->photo_us = 0;
        result = tool_result_command(name, valid &&
            control_actuator((uint16_t)angle->valueint, RP_PUSH_STOP));
    } else if (strcmp(name, "get_takeover_status") == 0) {
        agent_state_t agent;
        agent_runtime_state_snapshot(&agent);
        result = tool_result_ok(name);
        cJSON_AddStringToObject(result, "mode",
            agent.mode == AGENT_MODE_TAKEOVER ? "TAKEOVER" : "ASSIST");
        cJSON_AddNumberToObject(result, "lease_ms", agent.lease_remaining_ms);
    } else {
        result = tool_result_error("tool is not allowed");
    }
    cJSON_Delete(arguments);
    return result;
}

static void update_tool_summary(const char *name)
{
    if (s_status_mutex == NULL) {
        return;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    if (s_status.tool_summary[0] != '\0') {
        strlcat(s_status.tool_summary, ", ", sizeof(s_status.tool_summary));
    }
    strlcat(s_status.tool_summary, name, sizeof(s_status.tool_summary));
    xSemaphoreGive(s_status_mutex);
}

static bool process_response(cJSON *messages, cJSON *response, char *reply,
                             size_t reply_capacity, field_request_t *field)
{
    cJSON *choices = cJSON_GetObjectItem(response, "choices");
    cJSON *choice = cJSON_GetArrayItem(choices, 0);
    cJSON *message = cJSON_GetObjectItem(choice, "message");
    cJSON *content = cJSON_GetObjectItem(message, "content");
    cJSON *tool_calls = cJSON_GetObjectItem(message, "tool_calls");

    if (!cJSON_IsObject(message)) {
        set_error("DeepSeek response has no message");
        return false;
    }
    if (!cJSON_IsArray(tool_calls) ||
        cJSON_GetArraySize(tool_calls) == 0) {
        if (cJSON_IsString(content)) {
            strlcpy(reply, content->valuestring, reply_capacity);
        } else {
            strlcpy(reply, "模型没有返回文字内容。", reply_capacity);
        }
        return true;
    }

    cJSON *assistant = cJSON_Duplicate(message, true);
    cJSON_AddItemToArray(messages, assistant);
    int count = cJSON_GetArraySize(tool_calls);
    for (int i = 0; i < count; ++i) {
        cJSON *call = cJSON_GetArrayItem(tool_calls, i);
        cJSON *id = cJSON_GetObjectItem(call, "id");
        cJSON *function = cJSON_GetObjectItem(call, "function");
        cJSON *name = cJSON_GetObjectItem(function, "name");
        cJSON *arguments = cJSON_GetObjectItem(function, "arguments");
        cJSON *result = NULL;
        cJSON *image_content = NULL;

        if (!cJSON_IsString(id) || !cJSON_IsString(name)) {
            set_error("malformed tool call");
            return false;
        }
        agent_tool_class_t tool_class =
            agent_policy_tool_class(name->valuestring);
        if (tool_class == AGENT_TOOL_DENIED) {
            result = tool_result_error("tool is not allowed");
        } else if (tool_class == AGENT_TOOL_CONTROL &&
                   strcmp(name->valuestring, "field_decision") != 0 &&
                   !agent_runtime_allow_control(name->valuestring)) {
            result = tool_result_error(
                agent_runtime_mode() == AGENT_MODE_TAKEOVER ?
                "takeover lease is not valid; ask the operator to renew it" :
                "only one control action is allowed per request in ASSIST mode");
        } else if (strcmp(name->valuestring, "capture_photo") == 0) {
            image_content = capture_tool_content(field);
            if (image_content == NULL) {
                result = tool_result_error("fresh photo unavailable; stop and wait for pan");
            }
        } else {
            result = execute_tool(name->valuestring,
                                  cJSON_IsString(arguments) ?
                                  arguments->valuestring : "{}", field);
        }
        cJSON *tool_message = cJSON_CreateObject();
        cJSON_AddStringToObject(tool_message, "role", "tool");
        cJSON_AddStringToObject(tool_message, "tool_call_id",
                                id->valuestring);
        if (image_content != NULL) {
            cJSON_AddItemToObject(tool_message, "content", image_content);
        } else {
            char *result_text = cJSON_PrintUnformatted(result);
            cJSON_Delete(result);
            if (result_text == NULL) {
                cJSON_Delete(tool_message);
                set_error("tool result allocation failed");
                return false;
            }
            cJSON_AddStringToObject(tool_message, "content", result_text);
            cJSON_free(result_text);
        }
        cJSON_AddItemToArray(messages, tool_message);
        update_tool_summary(name->valuestring);
    }
    return false;
}

static void run_agent(const char *user_message)
{
    cJSON *messages = build_messages(user_message);
    cJSON *tools = build_tools();
    char reply[TD_AGENT_REPLY_CAP] = {0};
    bool finished = false;
    field_request_t field = {0};

    agent_runtime_begin_request();

    if (messages == NULL || tools == NULL) {
        set_error("agent memory allocation failed");
        goto finish;
    }

    for (int round = 0; round < AGENT_MAX_ROUNDS && !finished; ++round) {
        cJSON *request = build_request(messages, tools);
        char *response_text = NULL;
        int http_status = 0;
        esp_err_t err;
        cJSON *response;

        if (request == NULL) {
            set_error("request allocation failed");
            break;
        }
        int64_t call_start = app_now_us();
        err = call_deepseek(request, &response_text, &http_status);
        char http_detail[48];
        snprintf(http_detail, sizeof(http_detail), "HTTP %d, %ld ms", http_status,
                 (long)((app_now_us() - call_start) / 1000));
        rb_log_event("field", "cloud_request", http_detail);
        cJSON_Delete(request);
        if (err != ESP_OK) {
            char error[TD_AGENT_ERROR_CAP];
            if (http_status == 401) {
                strlcpy(error, "DeepSeek key invalid", sizeof(error));
            } else if (http_status == 429) {
                strlcpy(error, "DeepSeek rate limit", sizeof(error));
            } else {
                snprintf(error, sizeof(error), "DeepSeek HTTP %d: %s",
                         http_status, esp_err_to_name(err));
            }
            set_error(error);
            heap_caps_free(response_text);
            break;
        }
        response = cJSON_Parse(response_text);
        heap_caps_free(response_text);
        if (response == NULL) {
            set_error("DeepSeek returned malformed JSON");
            break;
        }
        if (http_status < 200 || http_status >= 300) {
            cJSON *error = cJSON_GetObjectItem(response, "error");
            cJSON *message = cJSON_GetObjectItem(error, "message");
            set_error(cJSON_IsString(message) ? message->valuestring :
                      "DeepSeek API error");
            cJSON_Delete(response);
            break;
        }
        finished = process_response(messages, response, reply, sizeof(reply), &field);
        cJSON_Delete(response);
        if (!finished && request_has_error()) {
            break;
        }
    }
finish:
    if (!finished && !request_has_error()) {
        set_error("DeepSeek tool loop exceeded limit");
    }
    if (!finished) {
        control_field_cancel();
    }

    char tools_used[sizeof(s_status.tool_summary)];
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    if (finished) {
        strlcpy(s_status.reply, reply, sizeof(s_status.reply));
        s_status.error[0] = '\0';
        ++s_status.requests;
    } else {
        strlcpy(reply, s_status.error, sizeof(reply));
    }
    strlcpy(tools_used, s_status.tool_summary, sizeof(tools_used));
    s_status.busy = false;
    s_status.done = true;
    xSemaphoreGive(s_status_mutex);
    agent_runtime_set_reply(user_message, reply, tools_used);
    cJSON_Delete(messages);
    cJSON_Delete(tools);
}

static void agent_task(void *arg)
{
    agent_request_t request;
    (void)arg;
    while (true) {
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) ==
            pdTRUE) {
            run_agent(request.text);
        }
    }
}

esp_err_t agent_service_start(void)
{
    esp_err_t err = load_api_key();
    if (err != ESP_OK) {
        return err;
    }
    s_status_mutex = xSemaphoreCreateMutex();
    s_request_queue = xQueueCreate(2u, sizeof(agent_request_t));
    if (s_status_mutex == NULL || s_request_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_status, 0, sizeof(s_status));
    s_status.configured = s_api_key[0] != '\0';
    /* TLS software big-number operations can run for several seconds.
     * Share the idle priority's time slices so idle/watchdog housekeeping
     * and every higher-priority control/UI task continue to run. */
    if (xTaskCreate(agent_task, "deepseek_agent", 12288, NULL, tskIDLE_PRIORITY,
                    NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t agent_service_set_api_key(const char *key)
{
    nvs_handle_t handle;
    esp_err_t err;
    if (!agent_policy_valid_api_key(key)) {
        return ESP_ERR_INVALID_ARG;
    }
    err = nvs_open(AGENT_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, AGENT_NVS_KEY_API, key);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err == ESP_OK) {
        if (s_status_mutex != NULL) {
            xSemaphoreTake(s_status_mutex, portMAX_DELAY);
            strlcpy(s_api_key, key, sizeof(s_api_key));
            s_status.configured = true;
            if (!s_status.busy) {
                s_status.error[0] = '\0';
            }
            xSemaphoreGive(s_status_mutex);
        } else {
            strlcpy(s_api_key, key, sizeof(s_api_key));
        }
    }
    return err;
}

bool agent_service_is_configured(void)
{
    if (s_status_mutex == NULL) {
        return s_api_key[0] != '\0';
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    bool configured = s_status.configured;
    xSemaphoreGive(s_status_mutex);
    return configured;
}

esp_err_t agent_service_submit(const char *message)
{
    agent_request_t request;
    size_t length;
    if (message == NULL || s_request_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    length = strlen(message);
    if (length == 0u || length >= sizeof(request.text)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (!agent_service_is_configured()) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(&request, 0, sizeof(request));
    strlcpy(request.text, message, sizeof(request.text));
    if (xSemaphoreTake(s_status_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_status.busy) {
        xSemaphoreGive(s_status_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_status.busy = true;
    s_status.done = false;
    s_status.reply[0] = '\0';
    s_status.error[0] = '\0';
    s_status.tool_summary[0] = '\0';
    ++s_status.sequence;
    xSemaphoreGive(s_status_mutex);
    if (xQueueSend(s_request_queue, &request, pdMS_TO_TICKS(1000)) !=
        pdTRUE) {
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.busy = false;
        s_status.done = true;
        strlcpy(s_status.error, "agent queue full",
                sizeof(s_status.error));
        xSemaphoreGive(s_status_mutex);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void agent_service_snapshot(td_agent_status_t *status)
{
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    if (s_status_mutex == NULL) {
        status->configured = agent_service_is_configured();
        return;
    }
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    *status = s_status;
    xSemaphoreGive(s_status_mutex);
}
