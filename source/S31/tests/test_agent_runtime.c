/*
 * Host test for the v3.10.0 agent runtime: mode/lease policy, per-request
 * action budget, plan executor and state/log JSON.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "agent/agent_runtime.h"
#include "bus/robot_bus.h"
#include "protocol/robot_protocol.h"

static int s_checks;
static int s_failures;
static uint32_t s_now_ms = 1000u;
static bool s_hw_ok = true;
static int s_stop_calls;
static int s_drive_calls;
static int s_pan_calls;
static int s_line_calls;
static int s_capture_calls;
static uint16_t s_last_pan;

#define CHECK(condition)                                                   \
    do {                                                                   \
        ++s_checks;                                                        \
        if (!(condition)) {                                                \
            ++s_failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
        }                                                                  \
    } while (0)

void rb_port_lock(void) {}
void rb_port_unlock(void) {}
uint32_t rb_port_now_ms(void) { return s_now_ms; }

/* Host stub for the platform hook implemented by agent_runtime_hw.c. */
bool agent_runtime_hw_safety_ok(void) { return s_hw_ok; }

static bool svc_stop(const void *req, size_t req_len, void *resp,
                     size_t *resp_len, void *ctx)
{
    (void)req; (void)req_len; (void)resp; (void)resp_len; (void)ctx;
    ++s_stop_calls;
    return true;
}

static bool svc_drive(const void *req, size_t req_len, void *resp,
                      size_t *resp_len, void *ctx)
{
    (void)req; (void)req_len; (void)resp; (void)resp_len; (void)ctx;
    ++s_drive_calls;
    return true;
}

static bool svc_set_pan(const void *req, size_t req_len, void *resp,
                        size_t *resp_len, void *ctx)
{
    (void)resp; (void)resp_len; (void)ctx;
    rp_actuator_cmd_t pan;
    if (req_len != RP_ACTUATOR_LEN ||
        !rp_unpack_actuator(req, RP_ACTUATOR_LEN, &pan)) {
        return false;
    }
    s_last_pan = pan.pan_deg;
    ++s_pan_calls;
    return true;
}

static bool svc_line(const void *req, size_t req_len, void *resp,
                     size_t *resp_len, void *ctx)
{
    (void)req; (void)req_len; (void)resp; (void)resp_len; (void)ctx;
    ++s_line_calls;
    return true;
}

static bool svc_capture(const void *req, size_t req_len, void *resp,
                        size_t *resp_len, void *ctx)
{
    (void)req; (void)req_len; (void)resp; (void)resp_len; (void)ctx;
    ++s_capture_calls;
    return true;
}

static bool apply_param(double value, void *ctx)
{
    (void)value; (void)ctx;
    return true;
}

static void register_stubs(void)
{
    (void)rb_topic_register("mode", "goumang/msg/AgentMode");
    (void)rb_topic_register("agent_event", "goumang/msg/AgentEvent");
    (void)rb_topic_register("line_state", "goumang/msg/LineState");
    (void)rb_topic_register("motor_state", "goumang/msg/MotorState");
    (void)rb_service_register("stop", svc_stop, NULL);
    (void)rb_service_register("drive", svc_drive, NULL);
    (void)rb_service_register("set_pan", svc_set_pan, NULL);
    (void)rb_service_register("line_start", svc_line, NULL);
    (void)rb_service_register("capture_photo", svc_capture, NULL);
    (void)rb_param_register("line_turn_gain", "int", 12, 0, 60,
                            apply_param, NULL);
}

static void test_mode_and_budget(void)
{
    agent_state_t state;

    agent_runtime_init();
    CHECK(agent_runtime_mode() == AGENT_MODE_ASSIST);
    CHECK(!agent_runtime_takeover_ok());

    agent_runtime_begin_request();
    CHECK(agent_runtime_allow_control("manual_move"));
    CHECK(!agent_runtime_allow_control("set_pan"));
    agent_runtime_begin_request();
    CHECK(agent_runtime_allow_control("set_pan"));

    /* Takeover requires the safety hook and a human call. */
    s_hw_ok = false;
    CHECK(!agent_runtime_set_takeover(true, 60u));
    CHECK(agent_runtime_mode() == AGENT_MODE_ASSIST);
    s_hw_ok = true;
    CHECK(agent_runtime_set_takeover(true, 60u));
    CHECK(agent_runtime_mode() == AGENT_MODE_TAKEOVER);
    CHECK(agent_runtime_takeover_ok());
    agent_runtime_state_snapshot(&state);
    CHECK(state.takeover);
    CHECK(state.lease_remaining_ms > 0u);
    CHECK(strcmp(state.last_action, "TAKEOVER ON") == 0);

    /* In takeover control is allowed while the lease is valid. */
    CHECK(agent_runtime_allow_control("manual_move"));
    CHECK(agent_runtime_allow_control("manual_move"));

    /* Human off stops and returns to ASSIST. */
    int stops_before = s_stop_calls;
    CHECK(agent_runtime_set_takeover(false, 0u));
    CHECK(agent_runtime_mode() == AGENT_MODE_ASSIST);
    CHECK(s_stop_calls > stops_before);
}

static void test_lease_expiry(void)
{
    agent_runtime_init();
    s_hw_ok = true;
    s_now_ms = 5000u;
    CHECK(agent_runtime_set_takeover(true, 10u));
    CHECK(agent_runtime_takeover_ok());
    s_now_ms += 9000u;
    agent_runtime_tick(s_now_ms);
    CHECK(agent_runtime_takeover_ok()); /* 9 s elapsed, lease still valid */
    int stops_before = s_stop_calls;
    s_now_ms += 2000u;
    agent_runtime_tick(s_now_ms);
    CHECK(agent_runtime_mode() == AGENT_MODE_ASSIST);
    CHECK(!agent_runtime_takeover_ok());
    CHECK(s_stop_calls > stops_before);
    CHECK(!agent_runtime_lease(30u));
}

static void test_plan_executor(void)
{
    agent_plan_step_t steps[4];
    char error[64] = {0};
    uint32_t plan_id = 0u;
    uint32_t plan_out = 0u;
    uint8_t total = 0u;
    uint8_t done = 0u;
    bool running = false;

    agent_runtime_init();
    s_hw_ok = true;
    s_now_ms = 20000u;
    s_pan_calls = 0;
    s_drive_calls = 0;
    s_capture_calls = 0;
    CHECK(agent_runtime_set_takeover(true, 60u));

    memset(steps, 0, sizeof(steps));
    steps[0].op = AGENT_STEP_PAN;
    steps[0].a = 540;
    steps[1].op = AGENT_STEP_WAIT;
    steps[1].c = 300;
    steps[2].op = AGENT_STEP_CAPTURE;
    steps[3].op = AGENT_STEP_PARAM;
    strcpy(steps[3].name, "line_turn_gain");
    steps[3].value = 20.0;

    CHECK(agent_runtime_submit_steps(steps, 4u, error, sizeof(error),
                                     &plan_id));
    CHECK(plan_id != 0u);
    CHECK(!agent_runtime_submit_steps(steps, 4u, error, sizeof(error),
                                      &plan_id));

    agent_runtime_tick(s_now_ms);
    CHECK(s_pan_calls == 1 && s_last_pan == 540u);
    s_now_ms += 100u;
    rp_motor_state_t motor = {.pan_deg = 540u, .pan_flags = RP_PAN_KNOWN | RP_PAN_MOVING};
    int motor_topic = rb_topic_find("motor_state");
    CHECK(rb_publish(motor_topic, &motor, sizeof(motor)));
    agent_runtime_tick(s_now_ms);
    CHECK(agent_runtime_plan_snapshot(NULL, NULL, &done, &running, NULL, 0u) && done == 0u);
    s_now_ms += 100u;
    motor.pan_flags = RP_PAN_KNOWN;
    CHECK(rb_publish(motor_topic, &motor, sizeof(motor)));
    agent_runtime_tick(s_now_ms);   /* pan arrived */
    agent_runtime_tick(s_now_ms);   /* wait starts */
    s_now_ms += 400u;
    agent_runtime_tick(s_now_ms);   /* wait finishes */
    agent_runtime_tick(s_now_ms);   /* capture */
    CHECK(s_capture_calls == 1);
    agent_runtime_tick(s_now_ms);   /* param */
    agent_runtime_tick(s_now_ms);   /* done */
    agent_runtime_plan_snapshot(&plan_out, &total, &done, &running, error,
                                sizeof(error));
    CHECK(plan_out == plan_id);
    CHECK(total == 4u);
    CHECK(done == 4u);
    CHECK(!running);

    /* A hardware fault aborts a running plan and stops the robot. */
    s_pan_calls = 0;
    s_now_ms = 30000u;
    int stops_before = s_stop_calls;
    steps[0].op = AGENT_STEP_DRIVE;
    steps[0].a = 10;
    steps[0].b = 10;
    steps[0].c = 500;
    CHECK(agent_runtime_submit_steps(steps, 1u, error, sizeof(error),
                                     &plan_id));
    agent_runtime_tick(s_now_ms);
    CHECK(s_drive_calls == 1);
    s_hw_ok = false;
    s_now_ms += 100u;
    agent_runtime_tick(s_now_ms);
    CHECK(s_stop_calls > stops_before);
    agent_runtime_plan_snapshot(&plan_out, &total, &done, &running, error,
                                sizeof(error));
    CHECK(!running);
    CHECK(strstr(error, "safety") != NULL);
}

static void test_json(void)
{
    char json[2048];
    agent_state_t state;

    agent_runtime_init();
    agent_runtime_set_reply("turn left", "turning left now", "manual_move");
    agent_runtime_state_snapshot(&state);
    CHECK(strcmp(state.last_user, "turn left") == 0);
    CHECK(strcmp(state.last_reply, "turning left now") == 0);
    CHECK(agent_runtime_state_json(json, sizeof(json)) > 0u);
    CHECK(strstr(json, "\"mode\":\"ASSIST\"") != NULL);
    CHECK(strstr(json, "last_reply") != NULL);
    CHECK(strstr(json, "lease_remaining_ms") != NULL);
    CHECK(agent_runtime_log_json(json, sizeof(json)) > 0u);
    CHECK(strstr(json, "entries") != NULL);
}

int main(void)
{
    rb_init();
    register_stubs();
    test_mode_and_budget();
    test_lease_expiry();
    test_plan_executor();
    test_json();
    printf("agent_runtime checks=%d failures=%d\n", s_checks, s_failures);
    return s_failures == 0 ? 0 : 1;
}