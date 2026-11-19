/*
 * Deterministic host simulation. S31 controller, agent runtime, bus and UART
 * codec run unchanged. The C30D plant supplies timed telemetry and executes
 * the production line-fusion and pan conversion functions.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "agent/agent_runtime.h"
#include "app/app_state.h"
#include "storage/c30d_nvm.h"
#include "control/control.h"
#include "freertos/semphr.h"
#include "line/line_fusion.h"
#include "pan/pan_param_math.h"
#include "bus/robot_bus.h"
#include "transport/robot_link.h"
#include "protocol/robot_protocol.h"
#include "protocol/s31_protocol.h"

#define STEP_MS 50u
#define PAN_STEPS_PER_180 1600

typedef struct {
    int8_t left;
    int8_t right;
    uint8_t line_speed;
    uint16_t range_cm;
    bool online;
    bool estop;
    bool drop_next_ack;
    bool inject_estop_on_clear;
    bool line_visible;
    int32_t pan_steps;
    int32_t pan_target;
    uint16_t pan_targets[8];
    unsigned pan_count;
    unsigned drive_count;
    unsigned stop_count;
    uint32_t last_motion_ms;
    line_motion_t line;
} simulated_c30d_t;

static simulated_c30d_t c30d;
static td_status_t status;
static uint32_t now_ms = 1000u;
static int64_t now_us = 1000000;
static unsigned checks;
static unsigned failures;
static unsigned graph_depth;
static bool suppress_sensor;
static bool suppress_safety;
static bool halt_before_next_tx;

#define CHECK(expr) do { \
    ++checks; \
    if (!(expr)) { ++failures; printf("  FAIL line %d: %s\n", __LINE__, #expr); } \
} while (0)

void control_sim_tick(void);

struct sim_mutex { bool held; };
static struct sim_mutex mutexes[2];
static unsigned mutex_count;

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    CHECK(mutex_count < sizeof(mutexes) / sizeof(mutexes[0]));
    return &mutexes[mutex_count++];
}

int xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t timeout)
{
    (void)timeout;
    if (mutex == &mutexes[1] && halt_before_next_tx) {
        halt_before_next_tx = false;
        CHECK(control_halt());
    }
    CHECK(!mutex->held);
    mutex->held = true;
    return pdTRUE;
}

int xSemaphoreGive(SemaphoreHandle_t mutex)
{
    CHECK(mutex->held);
    mutex->held = false;
    return pdTRUE;
}

void vSemaphoreDelete(SemaphoreHandle_t mutex) { (void)mutex; }
void app_state_lock(void) {}
void app_state_unlock(void) {}
td_status_t *app_state_mutable(void) { return &status; }
void app_state_snapshot(td_status_t *out)
{
    *out = status;
    out->stm32_online = c30d.online && status.stm32_last_rx_us > 0 &&
                       now_us - status.stm32_last_rx_us < 600000;
}
int64_t app_now_us(void) { return now_us; }
uint32_t app_now_ms(void) { return now_ms; }
void rb_port_lock(void) { ++graph_depth; }
void rb_port_unlock(void) { CHECK(graph_depth > 0); --graph_depth; }
uint32_t rb_port_now_ms(void) { return now_ms; }
/* Move the bytes across both production codecs in opposite directions. */
static bool transfer(bool from_c30d, uint8_t command, const uint8_t *payload,
                     uint8_t bytes, uint8_t decoded[RP_MAX_PAYLOAD])
{
    if (!c30d.online) {
        return false;
    }
    uint8_t frame[RP_MAX_FRAME];
    uint8_t received = 0;
    uint8_t received_size = 0;
    bool ready;

    if (from_c30d) {
        size_t length = s31_frame_encode(command, payload, bytes, frame);
        rp_parser_t parser;
        rp_parser_init(&parser);
        rp_parse_result_t result = RP_PARSE_NONE;
        for (size_t i = 0; i < length; ++i) {
            result = rp_parser_feed(&parser, frame[i], &received, decoded, &received_size);
        }
        ready = result == RP_PARSE_FRAME;
    } else {
        size_t length = rp_encode_frame(command, payload, bytes, frame, sizeof(frame));
        s31_frame_parser_t parser = {0};
        s31_frame_result_t result = S31_FRAME_NONE;
        for (size_t i = 0; i < length; ++i) {
            result = s31_frame_feed(&parser, frame[i]);
        }
        ready = result == S31_FRAME_READY;
        if (ready) {
            received = parser.command;
            received_size = parser.length;
            memcpy(decoded, parser.payload, parser.length);
        }
    }
    CHECK(ready && received == command && received_size == bytes);
    return ready && received == command && received_size == bytes;
}

static bool ack(uint8_t command, uint8_t result)
{
    rp_command_ack_t answer = {command, result};
    uint8_t payload[RP_COMMAND_ACK_LEN];
    uint8_t decoded[RP_MAX_PAYLOAD];
    if (c30d.drop_next_ack) {
        c30d.drop_next_ack = false;
        return false;
    }
    CHECK(rp_pack_command_ack(&answer, payload));
    CHECK(transfer(true, RP_CMD_COMMAND_ACK, payload, sizeof(payload), decoded));
    CHECK(rp_unpack_command_ack(decoded, sizeof(payload), &answer));
    return answer.command == command && answer.status == 0u;
}

bool robot_link_send_drive(int8_t left, int8_t right)
{
    rp_drive_cmd_t drive = {left, right};
    uint8_t payload[RP_DRIVE_LEN], decoded[RP_MAX_PAYLOAD];
    if (!rp_pack_drive(&drive, payload) || !transfer(false, RP_CMD_DRIVE, payload, sizeof(payload), decoded)) {
        return false;
    }
    CHECK(rp_unpack_drive(decoded, sizeof(payload), &drive));
    if (c30d.estop || line_motion_ultrasonic_guard(c30d.range_cm != 0u, c30d.range_cm,
                       drive.left_percent, drive.right_percent) == LINE_GUARD_STOP) {
        return ack(RP_CMD_DRIVE, 1u);
    }
    c30d.left = drive.left_percent;
    c30d.right = drive.right_percent;
    c30d.line_speed = 0;
    c30d.last_motion_ms = now_ms;
    c30d.drive_count += left != 0 || right != 0;
    return ack(RP_CMD_DRIVE, 0u);
}

bool robot_link_send_line_mode(bool enable, uint8_t speed)
{
    rp_line_cmd_t line = {enable, speed};
    uint8_t payload[RP_LINE_LEN], decoded[RP_MAX_PAYLOAD];
    if (!rp_pack_line_mode(&line, payload) || !transfer(false, RP_CMD_LINE, payload, sizeof(payload), decoded)) {
        return false;
    }
    CHECK(rp_unpack_line_mode(decoded, sizeof(payload), &line));
    if (c30d.estop) {
        return ack(RP_CMD_LINE, 1u);
    }
    c30d.line_speed = line.enable ? line.speed_percent : 0u;
    c30d.last_motion_ms = now_ms;
    return ack(RP_CMD_LINE, 0u);
}

bool robot_link_send_stop_all(void)
{
    uint8_t decoded[RP_MAX_PAYLOAD];
    if (!transfer(false, RP_CMD_STOP_ALL, NULL, 0, decoded)) {
        return false;
    }
    c30d.left = c30d.right = 0;
    c30d.line_speed = 0;
    ++c30d.stop_count;
    return ack(RP_CMD_STOP_ALL, 0u);
}

bool robot_link_emergency_stop(void)
{
    uint8_t decoded[RP_MAX_PAYLOAD];
    if (!transfer(false, RP_CMD_ESTOP, NULL, 0, decoded)) {
        return false;
    }
    c30d.estop = true;
    c30d.left = c30d.right = 0;
    c30d.line_speed = 0;
    c30d.pan_target = c30d.pan_steps;
    return true;
}

bool robot_link_clear_estop(void)
{
    uint8_t decoded[RP_MAX_PAYLOAD];
    if (!transfer(false, RP_CMD_CLEAR_ESTOP, NULL, 0, decoded)) {
        return false;
    }
    if (c30d.inject_estop_on_clear) {
        c30d.inject_estop_on_clear = false;
        CHECK(control_estop());
    }
    c30d.estop = false;
    return ack(RP_CMD_CLEAR_ESTOP, 0u);
}

bool robot_link_send_actuator(uint16_t angle, uint8_t push)
{
    rp_actuator_cmd_t actuator = {angle, push};
    uint8_t payload[RP_ACTUATOR_LEN], decoded[RP_MAX_PAYLOAD];
    if (!rp_pack_actuator(&actuator, payload) ||
        !transfer(false, RP_CMD_ACTUATOR, payload, sizeof(payload), decoded)) {
        return false;
    }
    CHECK(rp_unpack_actuator(decoded, sizeof(payload), &actuator));
    if (c30d.estop) {
        return ack(RP_CMD_ACTUATOR, 1u);
    }
    c30d.pan_target = pan_param_steps_from_deg(actuator.pan_deg, PAN_STEPS_PER_180);
    if (c30d.pan_count < sizeof(c30d.pan_targets) / sizeof(c30d.pan_targets[0])) {
        c30d.pan_targets[c30d.pan_count++] = actuator.pan_deg;
    }
    return ack(RP_CMD_ACTUATOR, 0u);
}

bool robot_link_send_calibrate_pan(void)
{
    c30d.pan_steps = c30d.pan_target = 0;
    return c30d.online && !c30d.estop;
}

bool robot_link_send_set_param(uint8_t id, int32_t value)
{
    return c30d.online && rp_param_valid(id, value);
}

static void publish_telemetry(void)
{
    if (!c30d.online) {
        return;
    }
    status.stm32_last_rx_us = now_us;
    if (!suppress_safety) {
        status.stm32_safety_rx_us = now_us;
        status.safety.robot_state = c30d.estop ? RP_ROBOT_ESTOP : RP_ROBOT_IDLE;
        status.safety.fault_flags = c30d.estop ? RP_FAULT_ESTOP : 0u;
    }
    if (!suppress_sensor) {
        status.stm32_sensor_rx_us = now_us;
        status.sensor.battery_mv = 12400;
        status.sensor.distance_cm = c30d.range_cm;
    }
    status.stm32_motor_rx_us = now_us;
    status.motor.pan_flags = RP_PAN_KNOWN |
        (c30d.pan_steps == c30d.pan_target ? 0u : RP_PAN_MOVING);
    status.motor.pan_deg = pan_param_deg_from_steps(c30d.pan_steps, PAN_STEPS_PER_180);
    status.motor.control_source = c30d.left != 0 || c30d.right != 0;
    status.motor.pwm_a = c30d.left;
    status.motor.pwm_b = c30d.right;
    status.motor.encoder_delta_a = c30d.left;
    status.motor.encoder_delta_b = c30d.right;
    uint8_t payload[RP_MOTOR_STATE_LEN], decoded[RP_MAX_PAYLOAD];
    CHECK(rp_pack_motor_state(&status.motor, payload));
    CHECK(transfer(true, RP_CMD_MOTOR_STATE, payload, sizeof(payload), decoded));
    CHECK(rp_unpack_motor_state(decoded, sizeof(payload), &status.motor));
    CHECK(rb_publish(rb_topic_find("motor_state"), &status.motor, sizeof(status.motor)));
    status.stm32_line_rx_us = now_us;
    status.line_state.lf_bits = c30d.line_visible ? 0x0Du : 0x0Fu;
    status.line_state.flags = c30d.line_speed != 0 ? RP_LINE_FLAG_ACTIVE : 0u;
    status.line_state.left_command = c30d.left;
    status.line_state.right_command = c30d.right;
    CHECK(rb_publish(rb_topic_find("line_state"), &status.line_state, sizeof(status.line_state)));
}

static void plant_step(void)
{
    if (now_ms - c30d.last_motion_ms > 500u) {
        c30d.left = c30d.right = 0;
        c30d.line_speed = 0;
    }
    int32_t delta = c30d.pan_target - c30d.pan_steps;
    /* 30 rpm at 3,200 pulses/revolution. */
    int32_t limit = 1600 * STEP_MS / 1000;
    c30d.pan_steps += delta > limit ? limit : (delta < -limit ? -limit : delta);
    if (c30d.line_speed != 0u) {
        line_motion_input_t input = {
            .enabled = true, .lf_bits = c30d.line_visible ? 0x0Du : 0x0Fu,
            .obstacle_valid = c30d.range_cm != 0u, .distance_cm = c30d.range_cm,
            .elapsed_ms = STEP_MS, .speed_percent = c30d.line_speed
        };
        line_motion_output_t output;
        line_motion_step(&c30d.line, &input, &output);
        c30d.left = output.left_percent;
        c30d.right = output.right_percent;
    }
}

static void advance(uint32_t milliseconds)
{
    while (milliseconds >= STEP_MS) {
        milliseconds -= STEP_MS;
        now_ms += STEP_MS;
        now_us += (int64_t)STEP_MS * 1000;
        plant_step();
        if (now_us % 100000 == 0) {
            publish_telemetry();
        }
        control_sim_tick();
        agent_runtime_tick(now_ms);
        rb_action_tick(now_ms);
    }
}

static bool graph_stop(const void *req, size_t len, void *out, size_t *bytes, void *ctx)
{
    (void)req; (void)len; (void)out; (void)bytes; (void)ctx;
    return control_stop();
}

static bool graph_drive(const void *req, size_t len, void *out, size_t *bytes, void *ctx)
{
    (void)out; (void)bytes; (void)ctx;
    const uint8_t *drive = req;
    return len == 4u && control_manual((int8_t)drive[0], (int8_t)drive[1],
                                      (uint16_t)(drive[2] | (drive[3] << 8)));
}

static rb_action_state_t graph_active(rb_goal_t *goal, void *ctx)
{
    (void)goal; (void)ctx;
    return RB_ACTION_ACTIVE;
}

static void test_frame_codec(void)
{
    puts("codec: C30D/S31 framing, CRC rejection and fixed emergency packet");
    uint8_t frame[RP_MAX_FRAME];
    const uint8_t emergency[] = {0xAAu, 0x55u, 0u, RP_CMD_ESTOP, 0x7Eu};
    CHECK(S31_MAX_FRAME == RP_MAX_FRAME);
    CHECK(s31_frame_encode(S31_CMD_ESTOP, NULL, 0, frame) == sizeof(emergency));
    CHECK(memcmp(frame, emergency, sizeof(emergency)) == 0);
    CHECK(s31_frame_encode(S31_CMD_DRIVE, NULL, 1, frame) == 0);

    s31_frame_parser_t parser = {0};
    s31_frame_result_t result = S31_FRAME_NONE;
    for (size_t i = 0; i < sizeof(emergency); ++i) {
        uint8_t byte = emergency[i];
        result = s31_frame_feed(&parser, i + 1 == sizeof(emergency) ? byte ^ 1u : byte);
    }
    CHECK(result == S31_FRAME_BAD_CRC);
    for (size_t i = 0; i < sizeof(emergency); ++i) {
        result = s31_frame_feed(&parser, emergency[i]);
    }
    CHECK(result == S31_FRAME_READY && parser.command == S31_CMD_ESTOP && parser.length == 0u);
}

static void test_field_guards(void)
{
    puts("field: line tracking, obstacle, Wi-Fi, per-group timeout");
    CHECK(control_field_move(FIELD_FOLLOW_LINE) == NULL);
    advance(200u);
    CHECK(c30d.line_speed == 15u && (c30d.left != 0 || c30d.right != 0));
    c30d.range_cm = 25u;
    advance(150u);
    CHECK(status.mode == TD_MODE_IDLE && c30d.left == 0 && c30d.right == 0);
    c30d.range_cm = 90u;
    advance(200u);
    CHECK(control_field_move(FIELD_FORWARD_SHORT) == NULL);
    advance(100u);
    CHECK(c30d.left == 12 && c30d.right == 12);
    status.sta_connected = false;
    advance(50u);
    CHECK(status.mode == TD_MODE_IDLE && c30d.left == 0);
    status.sta_connected = true;
    suppress_sensor = true;
    advance(700u);
    CHECK(status.stm32_last_rx_us == now_us);
    CHECK(control_field_move(FIELD_FORWARD_SHORT) != NULL);
    suppress_sensor = false;
    advance(200u);
    CHECK(control_field_move(FIELD_BACKWARD_SHORT) != NULL);
    CHECK(control_field_move(FIELD_TURN_LEFT_SMALL) == NULL);
    advance(100u);
    CHECK(c30d.left == -12 && c30d.right == 12);
    advance(400u);
    CHECK(status.mode == TD_MODE_IDLE && c30d.left == 0 && c30d.right == 0);
    advance(100u);
}

static void test_stop_and_ack(void)
{
    puts("transport: ACK loss, superseded send, ESTOP during clear, disconnect");
    unsigned stops = c30d.stop_count;
    advance(300u);
    CHECK(c30d.stop_count == stops);
    CHECK(control_manual(10, 10, 150u));
    advance(350u);
    CHECK(c30d.stop_count == stops + 1u && status.mode == TD_MODE_IDLE);
    advance(100u);

    c30d.drop_next_ack = true;
    CHECK(!control_manual(10, 10, 300u));
    advance(50u);
    CHECK(status.mode == TD_MODE_IDLE && c30d.left == 0);
    unsigned drives = c30d.drive_count;
    halt_before_next_tx = true;
    CHECK(!control_manual(10, 10, 300u));
    advance(150u);
    CHECK(c30d.drive_count == drives && c30d.left == 0);
    CHECK(control_estop());
    c30d.inject_estop_on_clear = true;
    CHECK(!control_clear_estop() && c30d.estop && status.mode == TD_MODE_ESTOP);
    CHECK(!control_manual(10, 10, 300u));
    CHECK(control_clear_estop());
    advance(600u);

    CHECK(control_manual(100, 100, 150u));
    CHECK(c30d.left == 100 && c30d.right == 100);
    advance(350u);
    CHECK(c30d.left == 0 && c30d.right == 0);

    CHECK(control_start_line_speed(100u));
    CHECK(c30d.line_speed == 100u);
    c30d.online = false;
    advance(550u);
    CHECK(status.mode == TD_MODE_IDLE && c30d.line_speed == 0);
    c30d.online = true;
    advance(200u);
    CHECK(c30d.line_speed == 0 && c30d.left == 0);
}

static void test_operator_plan(void)
{
    puts("automation: active-low line, STOP cancels queued work, lease expiry");
    agent_plan_step_t steps[2] = {
        {.op = AGENT_STEP_WAIT_LINE, .c = 1000},
        {.op = AGENT_STEP_DRIVE, .a = 12, .b = 12, .c = 1200}
    };
    c30d.line_visible = false;
    advance(150u);
    CHECK(agent_runtime_set_takeover(true, 10u));
    CHECK(agent_runtime_submit_steps(steps, 2u, NULL, 0, NULL));
    advance(250u);
    agent_state_t state;
    agent_runtime_state_snapshot(&state);
    CHECK(state.plan_running && state.plan_done == 0u);
    c30d.line_visible = true;
    advance(200u);
    CHECK(c30d.left == 12);
    uint32_t goal = rb_action_send("sim_action", NULL);
    CHECK(goal != 0u);
    CHECK(control_halt());
    unsigned drives = c30d.drive_count;
    advance(500u);
    rb_goal_t goal_state;
    CHECK(rb_action_status(goal, &goal_state) && goal_state.state == RB_ACTION_CANCELED);
    agent_runtime_state_snapshot(&state);
    CHECK(!state.plan_running && c30d.drive_count == drives && c30d.left == 0);

    now_ms = UINT32_MAX - 500u;
    CHECK(agent_runtime_set_takeover(true, 10u));
    CHECK(agent_runtime_lease_remaining_ms() == 10000u);
    advance(10100u);
    CHECK(agent_runtime_mode() == AGENT_MODE_ASSIST);
    CHECK(!agent_runtime_lease(10u));
}

static void test_pan(void)
{
    puts("pan: 540-degree bounds, persisted record, timed inspection feedback");
    CHECK(pan_param_steps_from_deg(0, PAN_STEPS_PER_180) == -2400);
    CHECK(pan_param_steps_from_deg(540, PAN_STEPS_PER_180) == 2400);
    C30D_NvmRecord saved = {
        .magic = C30D_NVM_MAGIC, .version = C30D_NVM_VERSION,
        .size = sizeof(saved), .pan_steps = 2400
    };
    c30d_nvm_set_crc(&saved);
    CHECK(c30d_nvm_pan_known(&saved, PAN_STEPS_PER_180));
    c30d.pan_steps = saved.pan_steps;
    c30d.pan_target = 0;
    advance(1550u);
    CHECK(status.motor.pan_deg == RP_PAN_HOME_DEG);
    saved.reserved = C30D_NVM_PAN_MOVING;
    c30d_nvm_set_crc(&saved);
    CHECK(c30d_nvm_valid(&saved) && !c30d_nvm_pan_known(&saved, PAN_STEPS_PER_180));
    CHECK(!control_actuator(541u, RP_PUSH_STOP));

    c30d.pan_steps = c30d.pan_target = 2400;
    c30d.pan_count = 0;
    advance(100u);
    CHECK(control_start_inspect());
    advance(1000u);
    CHECK(c30d.pan_count == 1u && c30d.pan_targets[0] == 270u);
    CHECK(status.mode == TD_MODE_INSPECT);
    advance(3900u);
    CHECK(c30d.pan_count == 4u && c30d.pan_targets[1] == 225u &&
          c30d.pan_targets[2] == 315u && c30d.pan_targets[3] == 270u);
    CHECK(status.mode == TD_MODE_IDLE && status.motor.pan_deg == 270u);
}

int main(void)
{
    status.sta_connected = true;
    c30d.online = true;
    c30d.line_visible = true;
    c30d.range_cm = 90u;
    line_motion_init(&c30d.line);
    rb_init();
    CHECK(rb_topic_register("line_state", "line") >= 0);
    CHECK(rb_topic_register("motor_state", "motor") >= 0);
    CHECK(rb_service_register("stop", graph_stop, NULL) >= 0);
    CHECK(rb_service_register("drive", graph_drive, NULL) >= 0);
    CHECK(rb_action_register("sim_action", graph_active, NULL) >= 0);
    CHECK(control_start() == ESP_OK);
    CHECK(agent_runtime_init());
    publish_telemetry();
    control_sim_tick();

    CHECK(app_rx_age_ms(600000, 0) == UINT32_MAX);
    CHECK(app_rx_age_ms(500000, 600000) == UINT32_MAX);
    CHECK(app_rx_age_ms(700000, 100001) == 599u);
    CHECK(app_rx_age_ms(700000, 100000) == 600u);
    test_frame_codec();
    test_field_guards();
    test_stop_and_ack();
    test_operator_plan();
    test_pan();
    CHECK(graph_depth == 0u);
    printf("simulation checks=%u failures=%u\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
