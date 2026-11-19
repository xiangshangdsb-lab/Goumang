/*
 * robot_services.c - the ROS2-like graph of the Goumang terminal.
 *
 * Registers the named topics, services, actions and parameters that the UI,
 * the web console and the agent all share.  This file is the only place that
 * maps graph names onto the legacy control_* functions.
 */
#include "robot_services.h"

#include <stdio.h>
#include <string.h>

#include "app_state.h"
#include "perception/camera_app.h"
#include "control/control.h"
#include "cJSON.h"
#include "esp_log.h"
#include "bus/robot_bus.h"
#include "protocol/robot_protocol.h"

static const char *TAG = "robot_srv";
static uint32_t s_capture_count;

typedef struct {
    int8_t left;
    int8_t right;
    uint16_t hold_ms;
} rb_drive_req_t;

typedef struct {
    uint8_t speed;
} rb_line_req_t;

typedef struct {
    uint8_t id;
    int32_t value;
} rb_param_req_t;

typedef struct {
    int8_t left;
    int8_t right;
    int32_t hold_ms;
} rb_move_action_t;

static const struct {
    const char *name;
    uint8_t id;
} s_param_map[] = {
    {"line_base_percent", RP_PARAM_LINE_BASE_PERCENT},
    {"line_turn_gain", RP_PARAM_LINE_TURN_GAIN},
    {"speed_kp", RP_PARAM_SPEED_KP},
    {"speed_ki", RP_PARAM_SPEED_KI},
    {"max_speed_mms", RP_PARAM_MAX_SPEED_MMS},
};

static bool svc_drive(const void *req, size_t req_len, void *resp,
                      size_t *resp_len, void *ctx)
{
    rb_drive_req_t command;
    td_status_t status;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    if (req == NULL || req_len != sizeof(command)) {
        return false;
    }
    memcpy(&command, req, sizeof(command));
    app_state_snapshot(&status);
    if (!status.stm32_online) {
        return false;
    }
    return control_manual(command.left, command.right, command.hold_ms);
}

static bool svc_stop(const void *req, size_t req_len, void *resp,
                     size_t *resp_len, void *ctx)
{
    (void)req;
    (void)req_len;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    return control_stop();
}

static bool svc_estop(const void *req, size_t req_len, void *resp,
                      size_t *resp_len, void *ctx)
{
    (void)req;
    (void)req_len;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    return control_estop();
}

static bool svc_clear_estop(const void *req, size_t req_len, void *resp,
                            size_t *resp_len, void *ctx)
{
    (void)req;
    (void)req_len;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    return control_clear_estop();
}

static bool svc_set_pan(const void *req, size_t req_len, void *resp,
                        size_t *resp_len, void *ctx)
{
    rp_actuator_cmd_t command;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    return req_len == RP_ACTUATOR_LEN &&
           rp_unpack_actuator(req, RP_ACTUATOR_LEN, &command) &&
           control_actuator(command.pan_deg, command.push_action);
}

static bool svc_line_start(const void *req, size_t req_len, void *resp,
                           size_t *resp_len, void *ctx)
{
    rb_line_req_t command;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    if (req == NULL || req_len != sizeof(command)) {
        return false;
    }
    memcpy(&command, req, sizeof(command));
    return control_start_line_speed(command.speed);
}

static bool svc_inspect_start(const void *req, size_t req_len, void *resp,
                              size_t *resp_len, void *ctx)
{
    (void)req;
    (void)req_len;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    return control_start_inspect();
}

static bool svc_capture_photo(const void *req, size_t req_len, void *resp,
                              size_t *resp_len, void *ctx)
{
    uint8_t *bmp = NULL;
    size_t length = 0u;
    (void)req;
    (void)req_len;
    (void)ctx;

    if (camera_app_get_bmp(&bmp, &length) != ESP_OK || bmp == NULL) {
        return false;
    }
    camera_app_free_bmp(bmp);
    ++s_capture_count;
    ESP_LOGI(TAG, "snapshot %u (%u bytes)", (unsigned)s_capture_count,
             (unsigned)length);
    return true;
}

static bool svc_set_param(const void *req, size_t req_len, void *resp,
                          size_t *resp_len, void *ctx)
{
    rb_param_req_t command;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    if (req == NULL || req_len != sizeof(command)) {
        return false;
    }
    memcpy(&command, req, sizeof(command));
    return control_set_param(command.id, command.value);
}

static bool param_apply(double value, void *ctx)
{
    uint8_t id = *(const uint8_t *)ctx;
    int32_t minimum;
    int32_t maximum;
    if (!rp_param_range(id, &minimum, &maximum, NULL) ||
        !(value >= minimum && value <= maximum) || value != (int32_t)value) {
        return false;
    }
    return control_set_param(id, (int32_t)value);
}

/* ---------------------------------------------------------------- actions */
static bool parse_action_args(const char *args, int32_t *a, int32_t *b,
                              int32_t *c)
{
    int first = 0;
    int second = 0;
    int third = 0;
    int count;

    if (args == NULL || args[0] == '\0') {
        return false;
    }
    count = sscanf(args, "%d,%d,%d", &first, &second, &third);
    if (count < 1) {
        return false;
    }
    *a = first;
    *b = count > 1 ? second : 0;
    *c = count > 2 ? third : 0;
    return true;
}

static rb_action_state_t action_move_for_ms(rb_goal_t *goal, void *ctx)
{
    rb_move_action_t *state = (rb_move_action_t *)ctx;
    uint32_t now = rb_port_now_ms();
    int32_t left = 0;
    int32_t right = 0;
    int32_t duration = 0;

    if (goal->progress == 0u) {
        if (!parse_action_args(goal->args, &left, &right, &duration)) {
            return RB_ACTION_FAILED;
        }
        state->left = (int8_t)(left < -40 ? -40 : (left > 40 ? 40 : left));
        state->right = (int8_t)(right < -40 ? -40 : (right > 40 ? 40 : right));
        state->hold_ms = duration < 100 ? 100 : (duration > 4000 ? 4000 : duration);
        goal->started_ms = now;
        goal->progress = 1u;
        strlcpy(goal->feedback, "moving", sizeof(goal->feedback));
    }
    if (now - goal->started_ms < (uint32_t)state->hold_ms) {
        return control_manual(state->left, state->right, 300u) ?
               RB_ACTION_ACTIVE : RB_ACTION_FAILED;
    }
    return control_stop() ? RB_ACTION_SUCCEEDED : RB_ACTION_FAILED;
}

static rb_action_state_t action_line_follow(rb_goal_t *goal, void *ctx)
{
    int32_t speed = 0;
    int32_t duration = 0;
    uint32_t now = rb_port_now_ms();
    (void)ctx;

    if (goal->progress == 0u) {
        int32_t ignored = 0;
        if (!parse_action_args(goal->args, &speed, &duration, &ignored)) {
            return RB_ACTION_FAILED;
        }
        if (speed < 5) speed = 5;
        if (speed > 40) speed = 40;
        if (duration < 200) duration = 200;
        goal->started_ms = now;
        goal->progress = 1u;
        if (!control_start_line_speed((uint8_t)speed)) {
            return RB_ACTION_FAILED;
        }
        strlcpy(goal->feedback, "line following", sizeof(goal->feedback));
    }
    if (now - goal->started_ms < (uint32_t)duration) {
        return RB_ACTION_ACTIVE;
    }
    return control_stop() ? RB_ACTION_SUCCEEDED : RB_ACTION_FAILED;
}

static rb_action_state_t action_patrol(rb_goal_t *goal, void *ctx)
{
    td_status_t status;
    (void)ctx;
    if (goal->progress == 0u) {
        goal->started_ms = rb_port_now_ms();
        goal->progress = 1u;
        if (!control_start_inspect()) {
            return RB_ACTION_FAILED;
        }
        strlcpy(goal->feedback, "patrol", sizeof(goal->feedback));
        return RB_ACTION_ACTIVE;
    }
    app_state_snapshot(&status);
    if (status.mode != TD_MODE_INSPECT) {
        return RB_ACTION_SUCCEEDED;
    }
    if (rb_port_now_ms() - goal->started_ms > 30000u) {
        (void)control_stop();
        return RB_ACTION_FAILED;
    }
    return RB_ACTION_ACTIVE;
}

static rb_move_action_t s_move_state;
static uint8_t s_patrol_state;

void robot_services_init(void)
{
    int topic;
    static const struct {
        const char *name;
        const char *type;
    } topics[] = {
        {"chassis_state", "goumang/msg/ChassisState"},
        {"sensor_state", "goumang/msg/SensorState"},
        {"safety_state", "goumang/msg/SafetyState"},
        {"line_state", "goumang/msg/LineState"},
        {"imu_state", "goumang/msg/ImuState"},
        {"motor_state", "goumang/msg/MotorState"},
        {"agent_event", "goumang/msg/AgentEvent"},
        {"mode", "goumang/msg/AgentMode"},
        {"param_state", "goumang/msg/ParamState"},
    };
    for (size_t i = 0; i < sizeof(topics) / sizeof(topics[0]); ++i) {
        topic = rb_topic_register(topics[i].name, topics[i].type);
        if (topic < 0) {
            ESP_LOGW(TAG, "topic %s register failed", topics[i].name);
        }
    }

    (void)rb_service_register("drive", svc_drive, NULL);
    (void)rb_service_register("stop", svc_stop, NULL);
    (void)rb_service_register("estop", svc_estop, NULL);
    (void)rb_service_register("clear_estop", svc_clear_estop, NULL);
    (void)rb_service_register("set_pan", svc_set_pan, NULL);
    (void)rb_service_register("line_start", svc_line_start, NULL);
    (void)rb_service_register("inspect_start", svc_inspect_start, NULL);
    (void)rb_service_register("capture_photo", svc_capture_photo, NULL);
    (void)rb_service_register("set_param", svc_set_param, NULL);

    (void)rb_action_register("move_for_ms", action_move_for_ms, &s_move_state);
    (void)rb_action_register("line_follow", action_line_follow, NULL);
    (void)rb_action_register("patrol", action_patrol, &s_patrol_state);

    for (size_t i = 0; i < sizeof(s_param_map) / sizeof(s_param_map[0]); ++i) {
        int32_t minimum;
        int32_t maximum;
        int32_t initial;
        uint8_t id = s_param_map[i].id;
        if (rp_param_range(id, &minimum, &maximum, &initial)) {
            (void)rb_param_register(s_param_map[i].name, "int", initial, minimum, maximum,
                                    param_apply, (void *)&s_param_map[i].id);
        }
    }
    s_capture_count = 0u;
    ESP_LOGI(TAG, "graph registered");
}

int robot_param_id_from_name(const char *name)
{
    if (name == NULL) {
        return -1;
    }
    for (size_t i = 0; i < sizeof(s_param_map) / sizeof(s_param_map[0]); ++i) {
        if (strcmp(name, s_param_map[i].name) == 0) {
            return (int)s_param_map[i].id;
        }
    }
    return -1;
}

size_t robot_params_json(char *buf, size_t cap)
{
    cJSON *root;
    char *text;
    size_t written = 0u;

    if (buf == NULL || cap == 0u) {
        return 0u;
    }
    root = cJSON_CreateArray();
    if (root == NULL) {
        buf[0] = '\0';
        return 0u;
    }
    for (size_t i = 0; i < sizeof(s_param_map) / sizeof(s_param_map[0]); ++i) {
        int32_t value = 0;
        int32_t min_value = 0;
        int32_t max_value = 0;
        int32_t default_value = 0;
        cJSON *item = cJSON_CreateObject();
        if (item == NULL) {
            continue;
        }
        (void)control_get_param(s_param_map[i].id, &value);
        (void)rp_param_range(s_param_map[i].id, &min_value, &max_value,
                             &default_value);
        cJSON_AddStringToObject(item, "name", s_param_map[i].name);
        cJSON_AddNumberToObject(item, "id", s_param_map[i].id);
        cJSON_AddNumberToObject(item, "value", value);
        cJSON_AddNumberToObject(item, "min", min_value);
        cJSON_AddNumberToObject(item, "max", max_value);
        cJSON_AddNumberToObject(item, "default", default_value);
        cJSON_AddItemToArray(root, item);
    }
    text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (text == NULL) {
        buf[0] = '\0';
        return 0u;
    }
    written = strlen(text);
    if (written >= cap) {
        written = cap - 1u;
    }
    memcpy(buf, text, written);
    buf[written] = '\0';
    cJSON_free(text);
    return written;
}

bool robot_param_set_by_name(const char *name, double value)
{
    int id = robot_param_id_from_name(name);
    if (id < 0) {
        return false;
    }
    return rb_param_set(s_param_map[id - 1].name, value);
}