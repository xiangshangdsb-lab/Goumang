/*
 * agent_runtime.c - on-device half of the AI agent.
 *
 * Owns the takeover mode, the control lease, the per-request action budget,
 * the multi-step plan executor and the reply/audit buffers.  The cloud model
 * can ask; this module decides.  Takeover can only be enabled by a human.
 */
#include "agent_runtime.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdatomic.h>
#include "app/app_config.h"
#ifndef AGENT_RUNTIME_HOST_TEST
#include "cJSON.h"
#endif
#include "bus/robot_bus.h"
#include "protocol/robot_protocol.h"

#ifndef AGENT_RUNTIME_HOST_TEST
#include "esp_log.h"
#define AR_LOG(...) ESP_LOGI("agent_rt", __VA_ARGS__)
#else
#define AR_LOG(...) ((void)0)
#endif

#define LEASE_MIN_S 10u
#define LEASE_MAX_S 120u
#define DRIVE_KEEPALIVE_MS 150u
#define DRIVE_HOLD_MS 400u
#define PAN_ARRIVAL_TIMEOUT_MS 4000u
#define STEP_DEFAULT_WAIT_MS 1000
#define PLAN_MAX_TOTAL_MS AGENT_MAX_PLAN_MS

typedef struct {
    agent_plan_step_t steps[AGENT_MAX_PLAN];
    uint8_t count;
    uint8_t index;
    bool running;
    bool step_started;
    uint32_t plan_id;
    uint32_t step_started_ms;
    uint32_t last_drive_ms;
    uint32_t deadline_ms;
    char error[64];
} agent_plan_t;

static agent_plan_t s_plan;
static _Atomic agent_mode_t s_mode = AGENT_MODE_ASSIST;
static uint32_t s_plan_next_id;
static _Atomic uint32_t s_lease_until_ms;
static uint32_t s_lease_s;
static uint32_t s_req_actions;
static uint32_t s_blocked;
static uint32_t s_actions_total;
static uint32_t s_now_ms;
static bool s_inited;
static void (*s_say_handler)(const char *text);

static char s_last_user[AGENT_MAX_USER];
static char s_last_reply[AGENT_MAX_REPLY];
static char s_last_tool[32];
static char s_last_action[40];

static void ar_copy(char *dst, const char *src, size_t cap)
{
    size_t len;
    if (dst == NULL || cap == 0u) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    len = strlen(src);
    if (len >= cap) {
        len = cap - 1u;
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static void json_escape(char *dst, size_t cap, const char *src)
{
    size_t out = 0u;
    if (dst == NULL || cap == 0u) {
        return;
    }
    dst[0] = '\0';
    if (src == NULL) {
        return;
    }
    for (const unsigned char *p = (const unsigned char *)src;
         *p != '\0' && out + 7u < cap; ++p) {
        unsigned char c = *p;
        if (c == '"' || c == '\\') {
            dst[out++] = '\\';
            dst[out++] = (char)c;
        } else if (c == '\n') {
            dst[out++] = '\\';
            dst[out++] = 'n';
        } else if (c == '\r') {
            dst[out++] = '\\';
            dst[out++] = 'r';
        } else if (c == '\t') {
            dst[out++] = '\\';
            dst[out++] = 't';
        } else if (c < 0x20u) {
            out += (size_t)snprintf(dst + out, cap - out, "\\u%04x", c);
        } else {
            dst[out++] = (char)c;
        }
    }
    dst[out] = '\0';
}

static uint32_t clamp_lease_s(uint32_t seconds)
{
    if (seconds < LEASE_MIN_S) {
        return LEASE_MIN_S;
    }
    if (seconds > LEASE_MAX_S) {
        return LEASE_MAX_S;
    }
    return seconds;
}

static bool plan_active(void)
{
    return s_plan.running;
}

static void publish_mode(void)
{
    uint8_t payload[2] = {(uint8_t)s_mode,
                          (uint8_t)(s_lease_until_ms > s_now_ms
                                        ? (s_lease_until_ms - s_now_ms) / 1000u
                                        : 0u)};
    int topic = rb_topic_find("mode");
    if (topic >= 0) {
        (void)rb_publish(topic, payload, sizeof(payload));
    }
}

static bool call_service(const char *name, const void *req, size_t req_len)
{
    return rb_service_call(name, req, req_len, NULL, NULL);
}

static void stop_robot(const char *why)
{
    (void)why;
    (void)call_service("stop", NULL, 0u);
    AR_LOG("stop: %s", why != NULL ? why : "policy");
}

static void plan_abort(const char *reason)
{
    if (s_plan.running) {
        ar_copy(s_plan.error, reason != NULL ? reason : "aborted",
                sizeof(s_plan.error));
        s_plan.running = false;
        snprintf(s_last_action, sizeof(s_last_action), "PLAN FAIL %u/%u",
                 (unsigned)s_plan.index + 1u, (unsigned)s_plan.count);
        agent_runtime_audit("agent", "plan_fail", reason);
    }
}

void agent_runtime_audit(const char *source, const char *event,
                         const char *detail)
{
    rb_log_event(source != NULL ? source : "agent",
                 event != NULL ? event : "",
                 detail != NULL ? detail : "");
}

void agent_runtime_set_reply(const char *user, const char *reply,
                             const char *tool)
{
    rb_port_lock();
    if (user != NULL) {
        ar_copy(s_last_user, user, sizeof(s_last_user));
    }
    if (reply != NULL) {
        ar_copy(s_last_reply, reply, sizeof(s_last_reply));
    }
    if (tool != NULL) {
        ar_copy(s_last_tool, tool, sizeof(s_last_tool));
    } else {
        s_last_tool[0] = '\0';
    }
    rb_port_unlock();
}

void agent_runtime_set_action(const char *action)
{
    rb_port_lock();
    if (action != NULL) {
        ar_copy(s_last_action, action, sizeof(s_last_action));
    } else {
        s_last_action[0] = '\0';
    }
    rb_port_unlock();
}

void agent_runtime_set_say_handler(void (*fn)(const char *text))
{
    s_say_handler = fn;
}

agent_mode_t agent_runtime_mode(void)
{
    return s_mode;
}

uint32_t agent_runtime_lease_remaining_ms(void)
{
    if (s_mode != AGENT_MODE_TAKEOVER) {
        return 0u;
    }
    uint32_t now = rb_port_now_ms();
    int32_t remaining = (int32_t)(s_lease_until_ms - now);
    return remaining > 0 ? (uint32_t)remaining : 0u;
}

bool agent_runtime_hw_safety_ok(void);

bool agent_runtime_takeover_ok(void)
{
    return s_mode == AGENT_MODE_TAKEOVER &&
           agent_runtime_lease_remaining_ms() > 0u &&
           agent_runtime_hw_safety_ok();
}

static void publish_agent_event(uint8_t kind, const char *text)
{
    uint8_t payload[RB_MAX_PAYLOAD];
    size_t n = 0u;
    payload[n++] = kind;
    if (text != NULL) {
        size_t len = strlen(text);
        if (len > sizeof(payload) - 1u) {
            len = sizeof(payload) - 1u;
        }
        memcpy(&payload[n], text, len);
        n += len;
    }
    int topic = rb_topic_find("agent_event");
    if (topic >= 0) {
        (void)rb_publish(topic, payload, n);
    }
}

bool agent_runtime_set_takeover(bool on, uint32_t lease_seconds)
{
    rb_port_lock();
    if (on) {
        if (!agent_runtime_hw_safety_ok()) {
            ++s_blocked;
            rb_port_unlock();
            return false;
        }
        s_mode = AGENT_MODE_TAKEOVER;
        s_lease_s = clamp_lease_s(lease_seconds);
        s_lease_until_ms = rb_port_now_ms() + s_lease_s * 1000u;
        s_req_actions = 0u;
        ar_copy(s_last_action, "TAKEOVER ON", sizeof(s_last_action));
        publish_mode();
        agent_runtime_audit("human", "takeover_on", "manual");
        publish_agent_event(1u, "takeover on");
    } else {
        agent_runtime_abort_motion("takeover off");
        stop_robot("takeover off");
        s_mode = AGENT_MODE_ASSIST;
        s_lease_until_ms = 0u;
        ar_copy(s_last_action, "TAKEOVER OFF", sizeof(s_last_action));
        publish_mode();
        agent_runtime_audit("human", "takeover_off", "manual");
        publish_agent_event(2u, "takeover off");
    }
    rb_port_unlock();
    return true;
}

bool agent_runtime_lease(uint32_t seconds)
{
    rb_port_lock();
    if (!agent_runtime_takeover_ok()) {
        rb_port_unlock();
        return false;
    }
    s_lease_s = clamp_lease_s(seconds);
    s_lease_until_ms = rb_port_now_ms() + s_lease_s * 1000u;
    publish_mode();
    rb_port_unlock();
    return true;
}

void agent_runtime_revoke(const char *reason)
{
    rb_port_lock();
    if (s_mode == AGENT_MODE_TAKEOVER) {
        agent_runtime_abort_motion(reason != NULL ? reason : "revoked");
        stop_robot(reason != NULL ? reason : "revoked");
        s_mode = AGENT_MODE_ASSIST;
        s_lease_until_ms = 0u;
        ar_copy(s_last_action, "REVOKED", sizeof(s_last_action));
        publish_mode();
        agent_runtime_audit("policy", "revoked",
                            reason != NULL ? reason : "");
    }
    rb_port_unlock();
}

void agent_runtime_abort_motion(const char *reason)
{
    rb_port_lock();
    plan_abort(reason);
    rb_action_cancel_all();
    rb_port_unlock();
}

void agent_runtime_begin_request(void)
{
    rb_port_lock();
    s_req_actions = 0u;
    rb_port_unlock();
}

bool agent_runtime_allow_control(const char *tool)
{
    bool allowed;
    rb_port_lock();
    if (!agent_runtime_hw_safety_ok()) {
        ++s_blocked;
        agent_runtime_audit("policy", "deny_safety", tool);
        rb_port_unlock();
        return false;
    }
    if (s_mode == AGENT_MODE_TAKEOVER) {
        allowed = agent_runtime_takeover_ok();
        if (!allowed) {
            ++s_blocked;
            agent_runtime_audit("policy", "deny_lease", tool);
        }
    } else if (s_req_actions == 0u) {
        allowed = true;
        ++s_req_actions;
    } else {
        allowed = false;
        ++s_blocked;
        agent_runtime_audit("policy", "deny_budget", tool);
    }
    if (allowed) {
        ++s_actions_total;
    }
    rb_port_unlock();
    return allowed;
}

/* ------------------------------------------------------------------ plan */
static bool submit_steps_locked(const agent_plan_step_t *steps, uint8_t count,
                                char *err, size_t err_cap)
{
    if (steps == NULL || count == 0u || count > AGENT_MAX_PLAN) {
        if (err != NULL) {
            ar_copy(err, "invalid step count", err_cap);
        } else {
            ar_copy(s_plan.error, "invalid step count", sizeof(s_plan.error));
        }
        return false;
    }
    memcpy(s_plan.steps, steps, sizeof(agent_plan_step_t) * count);
    s_plan.count = count;
    s_plan.index = 0u;
    s_plan.running = true;
    s_plan.step_started = false;
    s_plan.plan_id = ++s_plan_next_id;
    s_plan.error[0] = '\0';
    s_plan.deadline_ms = rb_port_now_ms() + PLAN_MAX_TOTAL_MS;
    ar_copy(s_last_action, "PLAN 1/", sizeof(s_last_action));
    return true;
}

bool agent_runtime_submit_steps(const agent_plan_step_t *steps, uint8_t count,
                                char *err, size_t err_cap, uint32_t *plan_id)
{
    bool ok;
    rb_port_lock();
    if (!agent_runtime_takeover_ok()) {
        ok = false;
        if (err != NULL) {
            ar_copy(err, "takeover with a valid lease is required", err_cap);
        }
    } else if (plan_active()) {
        ok = false;
        if (err != NULL) {
            ar_copy(err, "a plan is already running", err_cap);
        }
    } else {
        ok = submit_steps_locked(steps, count, err, err_cap);
        if (ok && plan_id != NULL) {
            *plan_id = s_plan.plan_id;
        }
    }
    rb_port_unlock();
    return ok;
}

bool agent_runtime_cancel_plan(uint32_t plan_id)
{
    rb_port_lock();
    if (!s_plan.running || (plan_id != 0u && plan_id != s_plan.plan_id)) {
        rb_port_unlock();
        return false;
    }
    plan_abort("canceled");
    stop_robot("plan canceled");
    rb_port_unlock();
    return true;
}

bool agent_runtime_plan_snapshot(uint32_t *plan_id, uint8_t *total,
                                 uint8_t *done, bool *running,
                                 char *err, size_t err_cap)
{
    rb_port_lock();
    if (plan_id != NULL) {
        *plan_id = s_plan.plan_id;
    }
    if (total != NULL) {
        *total = s_plan.count;
    }
    if (done != NULL) {
        *done = s_plan.index;
    }
    if (running != NULL) {
        *running = s_plan.running;
    }
    if (err != NULL && err_cap != 0u) {
        ar_copy(err, s_plan.error, err_cap);
    }
    rb_port_unlock();
    return true;
}

static void plan_next(void)
{
    ++s_plan.index;
    s_plan.step_started = false;
    if (s_plan.index >= s_plan.count) {
        s_plan.running = false;
        ar_copy(s_last_action, "PLAN DONE", sizeof(s_last_action));
        agent_runtime_audit("agent", "plan_done", "");
        publish_agent_event(3u, "plan done");
        return;
    }
    snprintf(s_last_action, sizeof(s_last_action), "PLAN %u/%u",
             (unsigned)s_plan.index + 1u, (unsigned)s_plan.count);
}

static void plan_fail(const char *why)
{
    plan_abort(why);
    stop_robot(why);
}

/* Returns true when the step finishes immediately or after the wait. */
static void run_step(agent_plan_step_t *step, uint32_t now)
{
    bool first_tick = !s_plan.step_started;
    if (first_tick) {
        s_plan.step_started = true;
        s_plan.step_started_ms = now;
        s_plan.last_drive_ms = 0u;
        publish_agent_event(4u, s_last_action);
    }
    switch (step->op) {
    case AGENT_STEP_PAN: {
        if (first_tick) {
            uint8_t req[RP_ACTUATOR_LEN];
            rp_actuator_cmd_t pan = {(uint16_t)step->a, RP_PUSH_STOP};
            if (step->a < 0 || step->a > RP_PAN_MAX_DEG ||
                !rp_pack_actuator(&pan, req) ||
                !call_service("set_pan", req, sizeof(req))) {
                plan_fail("set_pan failed");
            }
            break;
        }
        uint32_t elapsed_ms = now - s_plan.step_started_ms;
        if (elapsed_ms >= PAN_ARRIVAL_TIMEOUT_MS) {
            plan_fail("pan arrival timeout");
            break;
        }
        int topic = rb_topic_find("motor_state");
        rp_motor_state_t motor;
        size_t len;
        uint32_t max_age_ms = elapsed_ms < TD_STM32_ONLINE_MS ? elapsed_ms : TD_STM32_ONLINE_MS;
        uint8_t mask = RP_PAN_KNOWN | RP_PAN_MOVING | RP_PAN_BOOT_RETURN;
        if (max_age_ms > 0 &&
            rb_latest_recent(topic, &motor, sizeof(motor), &len, max_age_ms - 1u) &&
            len == sizeof(motor) && (motor.pan_flags & mask) == RP_PAN_KNOWN &&
            motor.pan_deg == (uint16_t)step->a) {
            plan_next();
        }
        break;
    }
    case AGENT_STEP_DRIVE: {
        uint32_t elapsed_ms = now - s_plan.step_started_ms;
        uint32_t duration_ms = step->c > 0 ? (uint32_t)step->c : 300u;
        if (elapsed_ms >= duration_ms) {
            if (call_service("stop", NULL, 0u)) {
                plan_next();
            } else {
                plan_fail("drive stop failed");
            }
            break;
        }
        if (first_tick || now - s_plan.last_drive_ms >= DRIVE_KEEPALIVE_MS) {
            uint16_t hold = (uint16_t)(duration_ms - elapsed_ms);
            if (hold > DRIVE_HOLD_MS) {
                hold = DRIVE_HOLD_MS;
            }
            int32_t left = step->a < -40 ? -40 : (step->a > 40 ? 40 : step->a);
            int32_t right = step->b < -40 ? -40 : (step->b > 40 ? 40 : step->b);
            const uint8_t req[4] = {
                (uint8_t)(int8_t)left, (uint8_t)(int8_t)right,
                (uint8_t)hold, (uint8_t)(hold >> 8)
            };
            if (!call_service("drive", req, sizeof(req))) {
                plan_fail("drive failed");
                break;
            }
            s_plan.last_drive_ms = now;
        }
        break;
    }
    case AGENT_STEP_LINE: {
        uint8_t speed = (uint8_t)(step->a < 5 ? 5 : (step->a > 40 ? 40 : step->a));
        uint32_t duration = step->c > 0 ? (uint32_t)step->c : 2000u;
        if (first_tick && !call_service("line_start", &speed, sizeof(speed))) {
            plan_fail("line_start failed");
        } else if (now - s_plan.step_started_ms >= duration) {
            if (call_service("stop", NULL, 0u)) {
                plan_next();
            } else {
                plan_fail("line stop failed");
            }
        }
        break;
    }
    case AGENT_STEP_WAIT: {
        int32_t duration = step->c > 0 ? step->c : STEP_DEFAULT_WAIT_MS;
        if (now - s_plan.step_started_ms >= (uint32_t)duration) {
            plan_next();
        }
        break;
    }
    case AGENT_STEP_WAIT_LINE: {
        int topic = rb_topic_find("line_state");
        rp_line_state_t line;
        size_t len = 0u;
        int32_t timeout = step->c > 0 ? step->c : 5000;
        bool found = false;
        if (topic >= 0 &&
            rb_latest_recent(topic, &line, sizeof(line), &len, TD_STM32_ONLINE_MS) &&
            len == sizeof(line)) {
            found = rp_line_has_signal(&line);
        }
        if (found) {
            plan_next();
        } else if (now - s_plan.step_started_ms >= (uint32_t)timeout) {
            plan_fail("wait_line timeout");
        }
        break;
    }
    case AGENT_STEP_PARAM: {
        if (step->name[0] == '\0') {
            plan_fail("param name empty");
        } else if (!rb_param_set(step->name, step->value)) {
            plan_fail("param rejected");
        } else {
            agent_runtime_audit("agent", "param", step->name);
            plan_next();
        }
        break;
    }
    case AGENT_STEP_CAPTURE: {
        if (!call_service("capture_photo", NULL, 0u)) {
            plan_fail("capture failed");
        } else {
            plan_next();
        }
        break;
    }
    case AGENT_STEP_SAY: {
        if (s_say_handler != NULL) {
            s_say_handler(step->text);
        } else {
            ar_copy(s_last_action, step->text, sizeof(s_last_action));
        }
        publish_agent_event(5u, step->text);
        plan_next();
        break;
    }
    case AGENT_STEP_STOP: {
        (void)call_service("stop", NULL, 0u);
        plan_next();
        break;
    }
    default:
        plan_fail("unknown step");
        break;
    }
}

bool agent_runtime_init(void)
{
    memset(&s_plan, 0, sizeof(s_plan));
    s_plan_next_id = 0u;
    s_mode = AGENT_MODE_ASSIST;
    s_lease_until_ms = 0u;
    s_lease_s = 0u;
    s_req_actions = 0u;
    s_blocked = 0u;
    s_actions_total = 0u;
    s_last_user[0] = '\0';
    s_last_reply[0] = '\0';
    s_last_tool[0] = '\0';
    ar_copy(s_last_action, "ASSIST", sizeof(s_last_action));
    s_now_ms = rb_port_now_ms();
    s_inited = true;
    publish_mode();
    agent_runtime_audit("system", "agent_runtime_init", TD_FW_VERSION);
    return true;
}

void agent_runtime_tick(uint32_t now_ms)
{
    if (!s_inited) {
        return;
    }
    rb_port_lock();
    s_now_ms = now_ms;
    if (s_mode == AGENT_MODE_TAKEOVER) {
        const char *reason = NULL;
        if (agent_runtime_lease_remaining_ms() == 0u) {
            reason = "lease expired";
        } else if (!agent_runtime_hw_safety_ok()) {
            reason = "safety fault";
        }
        if (reason != NULL) {
            agent_runtime_revoke(reason);
            rb_port_unlock();
            return;
        }
    }
    if (s_plan.running) {
        if ((int32_t)(now_ms - s_plan.deadline_ms) >= 0) {
            plan_fail("plan deadline");
        } else {
            run_step(&s_plan.steps[s_plan.index], now_ms);
        }
    }
    rb_port_unlock();
}

void agent_runtime_state_snapshot(agent_state_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    rb_port_lock();
    out->mode = s_mode;
    out->takeover = (s_mode == AGENT_MODE_TAKEOVER);
    out->lease_remaining_ms = agent_runtime_lease_remaining_ms();
    ar_copy(out->last_user, s_last_user, sizeof(out->last_user));
    ar_copy(out->last_reply, s_last_reply, sizeof(out->last_reply));
    ar_copy(out->last_tool, s_last_tool, sizeof(out->last_tool));
    ar_copy(out->last_action, s_last_action, sizeof(out->last_action));
    out->plan_total = s_plan.count;
    out->plan_done = s_plan.index;
    out->plan_running = s_plan.running;
    out->plan_id = s_plan.plan_id;
    ar_copy(out->plan_error, s_plan.error, sizeof(out->plan_error));
    out->blocked_actions = s_blocked;
    out->actions_total = s_actions_total;
    rb_port_unlock();
}

size_t agent_runtime_state_json(char *buf, size_t cap)
{
    agent_state_t st;
    char user[AGENT_MAX_USER * 2];
    char reply[AGENT_MAX_REPLY * 2];
    char tool[96];
    char action[96];
    char err[160];
    int n;

    if (buf == NULL || cap == 0u) {
        return 0u;
    }
    agent_runtime_state_snapshot(&st);
    json_escape(user, sizeof(user), st.last_user);
    json_escape(reply, sizeof(reply), st.last_reply);
    json_escape(tool, sizeof(tool), st.last_tool);
    json_escape(action, sizeof(action), st.last_action);
    json_escape(err, sizeof(err), st.plan_error);
    n = snprintf(buf, cap,
                 "{\"mode\":\"%s\",\"takeover\":%s,"
                 "\"lease_remaining_ms\":%u,\"last_user\":\"%s\","
                 "\"last_reply\":\"%s\",\"last_tool\":\"%s\","
                 "\"last_action\":\"%s\",\"plan\":{\"id\":%u,\"total\":%u,"
                 "\"done\":%u,\"running\":%s,\"error\":\"%s\"},"
                 "\"blocked_actions\":%u,\"actions_total\":%u,\"now_ms\":%u}",
                 st.mode == AGENT_MODE_TAKEOVER ? "TAKEOVER" : "ASSIST",
                 st.takeover ? "true" : "false",
                 (unsigned)st.lease_remaining_ms, user, reply, tool, action,
                 (unsigned)st.plan_id, (unsigned)st.plan_total,
                 (unsigned)st.plan_done, st.plan_running ? "true" : "false",
                 err, (unsigned)st.blocked_actions,
                 (unsigned)st.actions_total,
                 (unsigned)rb_port_now_ms());
    if (n < 0) {
        buf[0] = '\0';
        return 0u;
    }
    return (size_t)n < cap ? (size_t)n : cap - 1u;
}

size_t agent_runtime_log_json(char *buf, size_t cap)
{
    return rb_log_json(buf, cap);
}

#ifndef AGENT_RUNTIME_HOST_TEST
/* ------------------------------------------------------------ JSON plan */
static bool op_from_name(const char *name, agent_step_op_t *op)
{
    if (strcmp(name, "pan") == 0) {
        *op = AGENT_STEP_PAN;
    } else if (strcmp(name, "drive") == 0) {
        *op = AGENT_STEP_DRIVE;
    } else if (strcmp(name, "line") == 0) {
        *op = AGENT_STEP_LINE;
    } else if (strcmp(name, "wait") == 0) {
        *op = AGENT_STEP_WAIT;
    } else if (strcmp(name, "wait_line") == 0) {
        *op = AGENT_STEP_WAIT_LINE;
    } else if (strcmp(name, "param") == 0) {
        *op = AGENT_STEP_PARAM;
    } else if (strcmp(name, "capture") == 0) {
        *op = AGENT_STEP_CAPTURE;
    } else if (strcmp(name, "say") == 0) {
        *op = AGENT_STEP_SAY;
    } else if (strcmp(name, "stop") == 0) {
        *op = AGENT_STEP_STOP;
    } else {
        return false;
    }
    return true;
}

static int32_t json_int(cJSON *object, const char *key, int32_t fallback)
{
    cJSON *item = cJSON_GetObjectItem(object, key);
    return cJSON_IsNumber(item) && item->valuedouble == item->valueint ?
           item->valueint : fallback;
}

static const char *json_string(cJSON *object, const char *key)
{
    cJSON *item = cJSON_GetObjectItem(object, key);
    return cJSON_IsString(item) ? item->valuestring : NULL;
}

bool agent_runtime_submit_plan(const char *json, char *err, size_t err_cap,
                               uint32_t *plan_id)
{
    cJSON *root;
    cJSON *steps;
    agent_plan_step_t parsed[AGENT_MAX_PLAN];
    int count;
    int i;

    if (json == NULL) {
        if (err != NULL) {
            ar_copy(err, "empty plan", err_cap);
        }
        return false;
    }
    root = cJSON_ParseWithOpts(json, NULL, true);
    if (root == NULL) {
        if (err != NULL) {
            ar_copy(err, "plan is not valid JSON", err_cap);
        }
        return false;
    }
    steps = cJSON_GetObjectItem(root, "steps");
    if (!cJSON_IsArray(steps)) {
        cJSON_Delete(root);
        if (err != NULL) {
            ar_copy(err, "plan needs a steps array", err_cap);
        }
        return false;
    }
    count = cJSON_GetArraySize(steps);
    if (count <= 0 || count > AGENT_MAX_PLAN) {
        cJSON_Delete(root);
        if (err != NULL) {
            snprintf(err, err_cap, "step count must be 1..%u",
                     (unsigned)AGENT_MAX_PLAN);
        }
        return false;
    }
    memset(parsed, 0, sizeof(parsed));
    for (i = 0; i < count; ++i) {
        cJSON *item = cJSON_GetArrayItem(steps, i);
        const char *op_name = json_string(item, "op");
        agent_plan_step_t *step = &parsed[i];
        const char *text;
        int32_t duration = json_int(item, "ms", 0);

        if (!cJSON_IsObject(item) || op_name == NULL ||
            !op_from_name(op_name, &step->op)) {
            cJSON_Delete(root);
            if (err != NULL) {
                snprintf(err, err_cap, "step %d has an unknown op", i);
            }
            return false;
        }
        switch (step->op) {
        case AGENT_STEP_PAN:
            step->a = json_int(item, "deg", -1);
            if (step->a < 0 || step->a > RP_PAN_MAX_DEG) {
                cJSON_Delete(root);
                if (err != NULL) {
                    snprintf(err, err_cap, "step %d pan must be 0..540", i);
                }
                return false;
            }
            break;
        case AGENT_STEP_DRIVE:
            step->a = json_int(item, "left", 0);
            step->b = json_int(item, "right", 0);
            step->c = duration;
            if (step->a < -40 || step->a > 40 || step->b < -40 ||
                step->b > 40 || step->c < 100 || step->c > 4000) {
                cJSON_Delete(root);
                if (err != NULL) {
                    ar_copy(err, "drive needs left/right in -40..40 and ms 100..4000",
                            err_cap);
                }
                return false;
            }
            break;
        case AGENT_STEP_LINE:
            step->a = json_int(item, "speed", 25);
            step->c = duration;
            if (step->a < 5 || step->a > 40 || step->c < 200 ||
                step->c > 20000) {
                cJSON_Delete(root);
                if (err != NULL) {
                    ar_copy(err, "line needs speed 5..40 and ms 200..20000",
                            err_cap);
                }
                return false;
            }
            break;
        case AGENT_STEP_WAIT:
            step->c = duration > 0 ? duration : 500;
            if (step->c > 30000) {
                cJSON_Delete(root);
                if (err != NULL) {
                    ar_copy(err, "wait ms must be <= 30000", err_cap);
                }
                return false;
            }
            break;
        case AGENT_STEP_WAIT_LINE:
            step->c = duration > 0 ? duration : 5000;
            break;
        case AGENT_STEP_PARAM: {
            const char *name = json_string(item, "name");
            cJSON *value = cJSON_GetObjectItem(item, "value");
            if (name == NULL || !cJSON_IsNumber(value) ||
                strlen(name) >= sizeof(step->name)) {
                cJSON_Delete(root);
                if (err != NULL) {
                    ar_copy(err, "param needs name and numeric value", err_cap);
                }
                return false;
            }
            ar_copy(step->name, name, sizeof(step->name));
            step->value = value->valuedouble;
            break;
        }
        case AGENT_STEP_SAY:
            text = json_string(item, "text");
            if (text != NULL) {
                ar_copy(step->text, text, sizeof(step->text));
            }
            break;
        default:
            break;
        }
    }
    cJSON_Delete(root);
    return agent_runtime_submit_steps(parsed, (uint8_t)count, err, err_cap,
                                      plan_id);
}
#endif /* !AGENT_RUNTIME_HOST_TEST */
