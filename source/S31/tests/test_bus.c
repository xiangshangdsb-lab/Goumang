/*
 * test_bus.c - host tests for the robot_bus module.
 *
 * The module's only platform dependencies are rb_port_lock/unlock/now_ms, so
 * this file provides counting no-op implementations (plus a controllable
 * millisecond clock) and links against robot_bus.c only.  robot_bus_port.c
 * must NOT be linked here.
 *
 * Every public entry point must take rb_port_lock() exactly once and release
 * it on every exit path; LOCKED() asserts exactly that for non-nested calls,
 * NESTED() accepts the extra (recursive) locking a user callback produces.
 */

#include "bus/robot_bus.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;

#define CHECK(value)                                                       \
    do {                                                                   \
        ++checks;                                                          \
        if (!(value)) {                                                    \
            ++failures;                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);        \
        }                                                                  \
    } while (0)

/* ---------------------------------------------------------- host port mock */

static uint32_t g_now = 1000u;
static unsigned long g_lock_calls;
static unsigned long g_unlock_calls;
static int g_lock_depth;
static int g_max_lock_depth;

void rb_port_lock(void)
{
    ++g_lock_calls;
    ++g_lock_depth;
    if (g_lock_depth > g_max_lock_depth) {
        g_max_lock_depth = g_lock_depth;
    }
}

void rb_port_unlock(void)
{
    ++g_unlock_calls;
    --g_lock_depth;
}

uint32_t rb_port_now_ms(void)
{
    return g_now;
}

/* Exactly one lock/unlock pair, balanced, nothing left held. */
#define LOCKED(expr)                                                       \
    do {                                                                   \
        unsigned long lock0_ = g_lock_calls;                               \
        unsigned long unlock0_ = g_unlock_calls;                           \
        (void)(expr);                                                      \
        CHECK(g_lock_calls == lock0_ + 1ul);                               \
        CHECK(g_unlock_calls == unlock0_ + 1ul);                           \
        CHECK(g_lock_calls == g_unlock_calls);                             \
        CHECK(g_lock_depth == 0);                                          \
    } while (0)

/* At least one nested lock/unlock pair, balanced, nothing left held. */
#define NESTED(expr)                                                       \
    do {                                                                   \
        unsigned long lock0_ = g_lock_calls;                               \
        unsigned long unlock0_ = g_unlock_calls;                           \
        (void)(expr);                                                      \
        CHECK(g_lock_calls > lock0_);                                      \
        CHECK(g_unlock_calls > unlock0_);                                  \
        CHECK(g_lock_calls == g_unlock_calls);                             \
        CHECK(g_lock_depth == 0);                                          \
    } while (0)

/* ------------------------------------------------------------ test helpers */

static int count_substr(const char *haystack, const char *needle)
{
    const char *p = haystack;
    size_t n = strlen(needle);
    int count = 0;

    while (n > 0u && (p = strstr(p, needle)) != NULL) {
        ++count;
        p += n;
    }
    return count;
}

/* Conservative structural JSON validator: one root object, balanced
 * {} / [] nesting, well-formed string quoting, no control characters. */
static int json_valid(const char *text)
{
    int depth = 0;
    int in_string = 0;
    int escaped = 0;
    char last = '\0';
    const char *p;

    if (text == NULL || text[0] != '{') {
        return 0;
    }
    for (p = text; *p != '\0'; ++p) {
        unsigned char c = (unsigned char)*p;

        last = *p;
        if (in_string) {
            if (escaped) {
                escaped = 0;
            } else if (c == '\\') {
                escaped = 1;
            } else if (c == '"') {
                in_string = 0;
            } else if (c < 0x20u) {
                return 0;
            }
        } else {
            if (c == '"') {
                in_string = 1;
            } else if (c == '{' || c == '[') {
                ++depth;
            } else if (c == '}' || c == ']') {
                --depth;
                if (depth < 0) {
                    return 0;
                }
            } else if (c < 0x20u || c > 0x7eu) {
                return 0;
            }
        }
        if (depth > 64) {
            return 0;
        }
    }
    return !in_string && !escaped && depth == 0 && last == '}';
}

/* ---------------------------------------------------------------- topics */

typedef struct {
    int count;
    size_t last_len;
    unsigned char last[RB_MAX_PAYLOAD];
} sub_counter_t;

static void sub_count_cb(int topic, const void *data, size_t len, void *ctx)
{
    sub_counter_t *counter = (sub_counter_t *)ctx;

    (void)topic;
    ++counter->count;
    counter->last_len = len;
    if (len > 0u && len <= sizeof counter->last) {
        memcpy(counter->last, data, len);
    }
}

static void test_topics(void)
{
    const uint8_t v1[3] = {0xAAu, 0x01u, 0x02u};
    const uint8_t v2[5] = {1u, 2u, 3u, 4u, 5u};
    static char json[4096];
    sub_counter_t a;
    sub_counter_t b;
    sub_counter_t c;
    sub_counter_t d;
    sub_counter_t e;
    uint8_t out[RB_MAX_PAYLOAD];
    uint8_t tiny[2];
    size_t got;
    size_t rc;
    int index;
    bool ok;
    const char *name;

    memset(&a, 0, sizeof a);
    memset(&b, 0, sizeof b);
    memset(&c, 0, sizeof c);
    memset(&d, 0, sizeof d);
    memset(&e, 0, sizeof e);
    g_now = 1000u;

    LOCKED(rb_init());
    CHECK(rb_topic_find("gimbal/angle") == -1);
    LOCKED(index = rb_topic_find(NULL));
    CHECK(index == -1);
    LOCKED(name = rb_topic_name(0));
    CHECK(name == NULL);
    LOCKED(name = rb_topic_name(-1));
    CHECK(name == NULL);
    LOCKED(name = rb_topic_name(RB_MAX_TOPICS));
    CHECK(name == NULL);
    LOCKED(index = rb_topic_register(NULL, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_topic_register("", "t"));
    CHECK(index == -1);
    LOCKED(index = rb_topic_register("abcdefghijklmnopqrstuvwx", "t"));
    CHECK(index == -1); /* 24 chars: one too long */

    LOCKED(index = rb_topic_register("gimbal/angle", "std_msgs/Float64"));
    CHECK(index == 0);
    LOCKED(index = rb_topic_find("gimbal/angle"));
    CHECK(index == 0);
    LOCKED(name = rb_topic_name(0));
    CHECK(name != NULL && strcmp(name, "gimbal/angle") == 0);
    /* Duplicate registration is idempotent and keeps the first metadata. */
    LOCKED(index = rb_topic_register("gimbal/angle", "other/Type"));
    CHECK(index == 0);
    LOCKED(index = rb_topic_register("abcdefghijklmnopqrstuvw", "t"));
    CHECK(index == 1); /* 23 chars: max accepted length */
    LOCKED(index = rb_topic_find("abcdefghijklmnopqrstuvw"));
    CHECK(index == 1);
    LOCKED(index = rb_topic_find("abcdefghijklmnopqrstuvwx"));
    CHECK(index == -1);

    /* Nothing published yet. */
    got = 123u;
    LOCKED(ok = rb_latest(0, out, sizeof out, &got));
    CHECK(!ok);
    CHECK(got == 123u);
    LOCKED(ok = rb_latest(0, NULL, sizeof out, &got));
    CHECK(!ok);
    LOCKED(ok = rb_latest(-1, out, sizeof out, &got));
    CHECK(!ok);
    LOCKED(ok = rb_latest(RB_MAX_TOPICS, out, sizeof out, &got));
    CHECK(!ok);

    /* Publish, read back, overwrite. */
    LOCKED(ok = rb_publish(0, v1, sizeof v1));
    CHECK(ok);
    got = 0u;
    LOCKED(ok = rb_latest(0, out, sizeof out, &got));
    CHECK(ok && got == sizeof v1 && memcmp(out, v1, sizeof v1) == 0);
    LOCKED(ok = rb_publish(0, v2, sizeof v2));
    CHECK(ok);
    got = 0u;
    LOCKED(ok = rb_latest(0, out, sizeof out, &got));
    CHECK(ok && got == sizeof v2 && memcmp(out, v2, sizeof v2) == 0);
    /* rb_latest truncates to cap and reports the copied length. */
    got = 99u;
    LOCKED(ok = rb_latest(0, tiny, sizeof tiny, &got));
    CHECK(ok && got == sizeof tiny && tiny[0] == 1u && tiny[1] == 2u);
    /* NULL len is allowed. */
    LOCKED(ok = rb_latest(0, out, sizeof out, NULL));
    CHECK(ok);

    /* Publish rejection paths. */
    LOCKED(ok = rb_publish(-1, v1, sizeof v1));
    CHECK(!ok);
    LOCKED(ok = rb_publish(RB_MAX_TOPICS, v1, sizeof v1));
    CHECK(!ok);
    LOCKED(ok = rb_publish(0, v1, (size_t)RB_MAX_PAYLOAD + 1u));
    CHECK(!ok);
    LOCKED(ok = rb_publish(0, NULL, sizeof v1));
    CHECK(!ok);
    /* An empty payload is a legal latched value. */
    LOCKED(ok = rb_publish(0, NULL, 0u));
    CHECK(ok);
    got = 42u;
    LOCKED(ok = rb_latest(0, out, sizeof out, &got));
    CHECK(ok && got == 0u);

    /* Subscribers: each keeps its own count, all see the same payload. */
    LOCKED(ok = rb_subscribe(0, sub_count_cb, &a));
    CHECK(ok);
    LOCKED(ok = rb_subscribe(0, sub_count_cb, &b));
    CHECK(ok);
    LOCKED(ok = rb_publish(0, v1, sizeof v1));
    CHECK(ok);
    CHECK(a.count == 1 && a.last_len == sizeof v1 && a.last[0] == 0xAAu);
    CHECK(b.count == 1 && b.last_len == sizeof v1 && b.last[2] == 0x02u);
    LOCKED(ok = rb_publish(0, v2, sizeof v2));
    CHECK(ok);
    CHECK(a.count == 2 && a.last_len == sizeof v2 && a.last[4] == 5u);
    CHECK(b.count == 2 && b.last_len == sizeof v2);

    LOCKED(ok = rb_subscribe(0, sub_count_cb, &c));
    CHECK(ok);
    LOCKED(ok = rb_subscribe(0, sub_count_cb, &d));
    CHECK(ok);
    CHECK(c.count == 0 && d.count == 0);
    LOCKED(ok = rb_publish(0, v1, sizeof v1));
    CHECK(ok);
    CHECK(c.count == 1 && d.count == 1);
    /* Fifth subscriber is rejected; NULL callback / bad topic too. */
    LOCKED(ok = rb_subscribe(0, sub_count_cb, &e));
    CHECK(!ok);
    LOCKED(ok = rb_subscribe(0, NULL, &e));
    CHECK(!ok);
    LOCKED(ok = rb_subscribe(-1, sub_count_cb, &e));
    CHECK(!ok);
    LOCKED(ok = rb_subscribe(RB_MAX_TOPICS, sub_count_cb, &e));
    CHECK(!ok);

    /* Frequency statistics (graph JSON is the only public getter). */
    LOCKED(index = rb_topic_register("battery", "sensor_msgs/BatteryState"));
    CHECK(index == 2);
    g_now = 2000u;
    LOCKED(ok = rb_publish(index, v1, sizeof v1));
    CHECK(ok);
    g_now = 2100u;
    LOCKED(ok = rb_publish(index, v1, 2u));
    CHECK(ok);
    LOCKED(rc = rb_graph_json(json, sizeof json));
    CHECK(rc > 0u);
    CHECK(rc == strlen(json));
    CHECK(json_valid(json));
    CHECK(strstr(json, "\"name\":\"battery\"") != NULL);
    CHECK(strstr(json, "\"published\":2") != NULL);
    CHECK(strstr(json, "\"hz\":10.0") != NULL);
    CHECK(strstr(json, "\"subs\":0") != NULL);
    CHECK(strstr(json, "\"subs\":4") != NULL);
    /* The duplicate registration kept the original type string. */
    CHECK(strstr(json, "\"type\":\"std_msgs/Float64\"") != NULL);
    CHECK(strstr(json, "other/Type") == NULL);
}

static void test_topic_limits(void)
{
    char name[16];
    int index;
    int i;

    LOCKED(rb_init());
    for (i = 0; i < RB_MAX_TOPICS; ++i) {
        (void)snprintf(name, sizeof name, "t%02d", i);
        LOCKED(index = rb_topic_register(name, "t"));
        CHECK(index == i);
    }
    LOCKED(index = rb_topic_register("overflow", "t"));
    CHECK(index == -1);
    LOCKED(index = rb_topic_register("t00", "t"));
    CHECK(index == 0); /* duplicate still resolves when the table is full */
    LOCKED(index = rb_topic_find("t23"));
    CHECK(index == 23);
    LOCKED(index = rb_topic_register("abcdefghijklmnopqrstuvwx", "t"));
    CHECK(index == -1); /* name rejected before the full check */
}

/* -------------------------------------------------------------- services */

static int s_echo_calls;
static int s_fail_calls;

static bool svc_echo(const void *req, size_t req_len, void *resp,
                     size_t *resp_len, void *ctx)
{
    size_t cap = (resp_len != NULL) ? *resp_len : 0u;

    (void)ctx;
    ++s_echo_calls;
    if (req_len > cap) {
        return false;
    }
    if (resp != NULL && req != NULL && req_len > 0u) {
        memcpy(resp, req, req_len);
    }
    if (resp_len != NULL) {
        *resp_len = req_len;
    }
    return true;
}

static bool svc_fail(const void *req, size_t req_len, void *resp,
                     size_t *resp_len, void *ctx)
{
    (void)req;
    (void)req_len;
    (void)resp;
    (void)resp_len;
    (void)ctx;
    ++s_fail_calls;
    return false;
}

static void test_services(void)
{
    const uint8_t request[2] = {4u, 2u};
    uint8_t response[8];
    size_t response_len;
    int index;
    bool ok;
    const char *name;

    s_echo_calls = 0;
    s_fail_calls = 0;
    LOCKED(rb_init());

    LOCKED(index = rb_service_register("math/add", svc_echo, NULL));
    CHECK(index == 0);
    LOCKED(index = rb_service_register("math/add", svc_fail, NULL));
    CHECK(index == 0); /* duplicate keeps the first handler */
    LOCKED(index = rb_service_register("fault", svc_fail, NULL));
    CHECK(index == 1);
    LOCKED(index = rb_service_register(NULL, svc_echo, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_service_register("", svc_echo, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_service_register("abcdefghijklmnopqrstuvwx", svc_echo,
                                       NULL));
    CHECK(index == -1);
    LOCKED(index = rb_service_register("nullcb", NULL, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_service_find("math/add"));
    CHECK(index == 0);
    LOCKED(index = rb_service_find("nope"));
    CHECK(index == -1);
    LOCKED(index = rb_service_find(NULL));
    CHECK(index == -1);
    LOCKED(name = rb_service_name(0));
    CHECK(name != NULL && strcmp(name, "math/add") == 0);
    LOCKED(name = rb_service_name(-1));
    CHECK(name == NULL);
    LOCKED(name = rb_service_name(RB_MAX_SERVICES));
    CHECK(name == NULL);

    /* Normal call: output copied, length reported. */
    memset(response, 0, sizeof response);
    response_len = sizeof response;
    LOCKED(ok = rb_service_call("math/add", request, sizeof request, response,
                                &response_len));
    CHECK(ok);
    CHECK(s_echo_calls == 1);
    CHECK(response_len == sizeof request);
    CHECK(response[0] == 4u && response[1] == 2u);

    /* Unknown service. */
    response_len = sizeof response;
    LOCKED(ok = rb_service_call("nope", request, sizeof request, response,
                                &response_len));
    CHECK(!ok);
    CHECK(s_echo_calls == 1);

    /* Handler error. */
    response_len = 0u;
    LOCKED(ok = rb_service_call("fault", request, sizeof request, response,
                                &response_len));
    CHECK(!ok);
    CHECK(s_fail_calls == 1);

    /* Caller capacity too small: the handler rejects the call. */
    response_len = 1u;
    LOCKED(ok = rb_service_call("math/add", request, sizeof request, response,
                                &response_len));
    CHECK(!ok);
    CHECK(s_echo_calls == 2);

    /* Empty request and NULL response length. */
    LOCKED(ok = rb_service_call("math/add", NULL, 0u, NULL, NULL));
    CHECK(ok);
    CHECK(s_echo_calls == 3);
    LOCKED(ok = rb_service_call(NULL, NULL, 0u, NULL, NULL));
    CHECK(!ok);
}

static void test_service_limits(void)
{
    char name[16];
    int index;
    int i;

    LOCKED(rb_init());
    for (i = 0; i < RB_MAX_SERVICES; ++i) {
        (void)snprintf(name, sizeof name, "svc%02d", i);
        LOCKED(index = rb_service_register(name, svc_fail, NULL));
        CHECK(index == i);
    }
    LOCKED(index = rb_service_register("overflow", svc_fail, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_service_register("svc00", svc_fail, NULL));
    CHECK(index == 0);
}

/* --------------------------------------------------------------- actions */

static int s_step_calls;
static int s_step_succeed_at;
static int s_reenter_depth_seen;
static bool s_step_reenter;
static int s_fail_step_calls;

static rb_action_state_t step_goto(rb_goal_t *goal, void *ctx)
{
    (void)ctx;
    ++s_step_calls;
    if (s_step_reenter) {
        rb_goal_t probe;

        (void)rb_action_status(goal->id, &probe);
        rb_log_event("step", "reenter", "nested");
        s_reenter_depth_seen = g_max_lock_depth;
    }
    goal->progress =
        (uint8_t)((s_step_calls * 50 > 100) ? 100 : s_step_calls * 50);
    (void)snprintf(goal->feedback, sizeof goal->feedback, "step %d",
                   s_step_calls);
    if (s_step_calls >= s_step_succeed_at) {
        return RB_ACTION_SUCCEEDED;
    }
    return RB_ACTION_ACTIVE;
}

static rb_action_state_t step_fail(rb_goal_t *goal, void *ctx)
{
    (void)ctx;
    ++s_fail_step_calls;
    (void)snprintf(goal->feedback, sizeof goal->feedback, "boom");
    return RB_ACTION_FAILED;
}

static void test_actions(void)
{
    rb_goal_t goal;
    uint32_t goal_id;
    uint32_t other_id;
    int index;
    bool ok;

    LOCKED(rb_init());
    g_now = 500u;
    s_step_calls = 0;
    s_step_succeed_at = 2;
    s_step_reenter = false;
    s_fail_step_calls = 0;
    s_reenter_depth_seen = 0;
    memset(&goal, 0, sizeof goal);

    LOCKED(index = rb_action_register("goto", step_goto, NULL));
    CHECK(index == 0);
    LOCKED(index = rb_action_register("goto", step_fail, NULL));
    CHECK(index == 0); /* duplicate keeps the first step fn */
    LOCKED(index = rb_action_register("fail", step_fail, NULL));
    CHECK(index == 1);
    LOCKED(index = rb_action_register(NULL, step_goto, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_action_register("", step_goto, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_action_register("abcdefghijklmnopqrstuvwx", step_goto,
                                      NULL));
    CHECK(index == -1);
    LOCKED(index = rb_action_register("nulstep", NULL, NULL));
    CHECK(index == -1);

    /* Unknown action -> goal id 0. */
    LOCKED(goal_id = rb_action_send("nope", NULL));
    CHECK(goal_id == 0u);
    LOCKED(goal_id = rb_action_send(NULL, NULL));
    CHECK(goal_id == 0u);

    LOCKED(goal_id = rb_action_send("goto", "{\"deg\":90}"));
    CHECK(goal_id == 1u);
    LOCKED(ok = rb_action_status(goal_id, &goal));
    CHECK(ok);
    CHECK(goal.state == RB_ACTION_ACTIVE);
    CHECK(strcmp(goal.name, "goto") == 0);
    CHECK(strcmp(goal.args, "{\"deg\":90}") == 0);
    CHECK(goal.started_ms == 500u);
    CHECK(goal.updated_ms == 500u);
    CHECK(goal.progress == 0u);
    CHECK(goal.feedback[0] == '\0');
    LOCKED(ok = rb_action_status(0u, &goal));
    CHECK(!ok);
    LOCKED(ok = rb_action_status(999u, &goal));
    CHECK(!ok);
    LOCKED(ok = rb_action_status(goal_id, NULL));
    CHECK(!ok);
    /* One goal per action: a second send while ACTIVE is refused. */
    LOCKED(other_id = rb_action_send("goto", "{}"));
    CHECK(other_id == 0u);

    /* Tick advances an active goal (feedback/progress written by the step). */
    g_now = 550u;
    LOCKED(rb_action_tick(g_now));
    LOCKED(ok = rb_action_status(goal_id, &goal));
    CHECK(ok);
    CHECK(s_step_calls == 1);
    CHECK(goal.state == RB_ACTION_ACTIVE);
    CHECK(goal.progress == 50u);
    CHECK(strcmp(goal.feedback, "step 1") == 0);
    CHECK(goal.updated_ms == 550u);

    /* Second tick completes the goal and stops calling the step. */
    g_now = 600u;
    LOCKED(rb_action_tick(g_now));
    LOCKED(ok = rb_action_status(goal_id, &goal));
    CHECK(ok);
    CHECK(s_step_calls == 2);
    CHECK(goal.state == RB_ACTION_SUCCEEDED);
    CHECK(goal.updated_ms == 600u);
    LOCKED(rb_action_tick(650u));
    CHECK(s_step_calls == 2);
    LOCKED(rb_action_tick(660u));
    CHECK(s_step_calls == 2);

    /* A finished slot accepts a new goal with a fresh id. */
    s_step_calls = 0;
    s_step_succeed_at = 1;
    LOCKED(other_id = rb_action_send("goto", "{\"deg\":45}"));
    CHECK(other_id == 2u);
    LOCKED(ok = rb_action_status(goal_id, &goal));
    CHECK(!ok); /* the previous goal id is no longer queryable */
    LOCKED(ok = rb_action_status(other_id, &goal));
    CHECK(ok && goal.state == RB_ACTION_ACTIVE);

    /* Cancel. */
    g_now = 650u;
    LOCKED(ok = rb_action_cancel(other_id));
    CHECK(ok);
    LOCKED(ok = rb_action_status(other_id, &goal));
    CHECK(ok && goal.state == RB_ACTION_CANCELED && goal.updated_ms == 650u);
    LOCKED(ok = rb_action_cancel(other_id));
    CHECK(!ok); /* already canceled */
    LOCKED(ok = rb_action_cancel(12345u));
    CHECK(!ok); /* unknown goal */
    LOCKED(ok = rb_action_cancel(0u));
    CHECK(!ok);

    /* Failing action. */
    LOCKED(other_id = rb_action_send("fail", "{}"));
    CHECK(other_id == 3u);
    g_now = 700u;
    LOCKED(rb_action_tick(g_now));
    LOCKED(ok = rb_action_status(other_id, &goal));
    CHECK(ok && s_fail_step_calls == 1 && goal.state == RB_ACTION_FAILED);

    /* Nested callback: a step that calls back into the bus needs a recursive
     * lock on target; on host the depth counter proves the nesting exists. */
    s_step_reenter = true;
    s_step_calls = 0;
    s_step_succeed_at = 1;
    s_reenter_depth_seen = 0;
    LOCKED(other_id = rb_action_send("goto", "{\"deg\":0}"));
    CHECK(other_id == 4u);
    g_max_lock_depth = 0;
    NESTED(rb_action_tick(750u));
    CHECK(s_step_calls == 1);
    CHECK(s_reenter_depth_seen >= 2);
    LOCKED(ok = rb_action_status(other_id, &goal));
    CHECK(ok && goal.state == RB_ACTION_SUCCEEDED);
    s_step_reenter = false;
}

static void test_action_limits(void)
{
    char name[16];
    int index;
    int i;

    LOCKED(rb_init());
    for (i = 0; i < RB_MAX_ACTIONS; ++i) {
        (void)snprintf(name, sizeof name, "act%02d", i);
        LOCKED(index = rb_action_register(name, step_fail, NULL));
        CHECK(index == i);
    }
    LOCKED(index = rb_action_register("overflow", step_fail, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_action_register("act00", step_fail, NULL));
    CHECK(index == 0);
}

/* ---------------------------------------------------------------- params */

static int s_apply_calls;
static double s_apply_last;
static double s_apply_reject;

static bool apply_speed(double value, void *ctx)
{
    (void)ctx;
    ++s_apply_calls;
    s_apply_last = value;
    return value != s_apply_reject;
}

static bool apply_always_fail(double value, void *ctx)
{
    (void)value;
    (void)ctx;
    return false;
}

static void test_params(void)
{
    double value;
    int index;
    bool ok;
    const char *name;

    LOCKED(rb_init());
    s_apply_calls = 0;
    s_apply_last = 0.0;
    s_apply_reject = -999.0;

    LOCKED(index = rb_param_register("speed", "double", 30.0, 0.0, 100.0,
                                     apply_speed, NULL));
    CHECK(index == 0);
    CHECK(s_apply_calls == 1);
    CHECK(s_apply_last == 30.0); /* default applied at registration */
    LOCKED(index = rb_param_register("speed", "int", 1.0, 0.0, 2.0,
                                     apply_speed, NULL));
    CHECK(index == 0); /* duplicate: existing config kept, no extra apply */
    CHECK(s_apply_calls == 1);
    LOCKED(index = rb_param_register("plain", NULL, 0.5, 0.0, 1.0, NULL,
                                     NULL));
    CHECK(index == 1);
    CHECK(strcmp(rb_param_name(0), "speed") == 0);
    LOCKED(name = rb_param_name(1));
    CHECK(name != NULL && strcmp(name, "plain") == 0);
    LOCKED(name = rb_param_name(-1));
    CHECK(name == NULL);
    LOCKED(name = rb_param_name(RB_MAX_PARAMS));
    CHECK(name == NULL);
    LOCKED(index = rb_param_find("speed"));
    CHECK(index == 0);
    LOCKED(index = rb_param_find("nope"));
    CHECK(index == -1);
    LOCKED(index = rb_param_find(NULL));
    CHECK(index == -1);
    LOCKED(ok = rb_param_get("speed", &value));
    CHECK(ok && value == 30.0);
    LOCKED(ok = rb_param_get("nope", &value));
    CHECK(!ok);
    LOCKED(ok = rb_param_get("speed", NULL));
    CHECK(!ok);

    /* In-range set: apply hook runs, then the value commits. */
    LOCKED(ok = rb_param_set("speed", 75.0));
    CHECK(ok);
    CHECK(s_apply_calls == 2);
    CHECK(s_apply_last == 75.0);
    LOCKED(ok = rb_param_get("speed", &value));
    CHECK(ok && value == 75.0);

    /* Out of range: rejected BEFORE the apply hook runs. */
    LOCKED(ok = rb_param_set("speed", 101.0));
    CHECK(!ok);
    LOCKED(ok = rb_param_set("speed", -0.5));
    CHECK(!ok);
    CHECK(s_apply_calls == 2);
    LOCKED(ok = rb_param_get("speed", &value));
    CHECK(ok && value == 75.0);

    /* Non-finite values are rejected too. */
    LOCKED(ok = rb_param_set("speed", NAN));
    CHECK(!ok);
    LOCKED(ok = rb_param_set("speed", HUGE_VAL));
    CHECK(!ok);
    LOCKED(ok = rb_param_set("speed", -HUGE_VAL));
    CHECK(!ok);
    CHECK(s_apply_calls == 2);

    /* In range but the apply hook refuses: old value stays. */
    s_apply_reject = 42.0;
    LOCKED(ok = rb_param_set("speed", 42.0));
    CHECK(!ok);
    CHECK(s_apply_calls == 3);
    CHECK(s_apply_last == 42.0);
    LOCKED(ok = rb_param_get("speed", &value));
    CHECK(ok && value == 75.0);
    s_apply_reject = -999.0;

    /* Unknown name. */
    LOCKED(ok = rb_param_set("nope", 1.0));
    CHECK(!ok);

    /* Param without an apply hook. */
    LOCKED(ok = rb_param_set("plain", 0.75));
    CHECK(ok);
    LOCKED(ok = rb_param_get("plain", &value));
    CHECK(ok && value == 0.75);

    /* Invalid registrations. */
    LOCKED(index = rb_param_register("bad_range", "d", 5.0, 10.0, 0.0, NULL,
                                     NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register("bad_default", "d", 50.0, 0.0, 10.0, NULL,
                                     NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register("bad_nan", "d", NAN, 0.0, 10.0, NULL,
                                     NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register("bad_inf", "d", 1.0, 0.0, HUGE_VAL, NULL,
                                     NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register("bad_apply", "d", 0.0, 0.0, 10.0,
                                     apply_always_fail, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_find("bad_apply"));
    CHECK(index == -1); /* failed registration did not take a slot */
    LOCKED(index = rb_param_register("", "d", 0.0, 0.0, 1.0, NULL, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register(NULL, "d", 0.0, 0.0, 1.0, NULL, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register("abcdefghijklmnopqrstuvwx", "d", 0.0,
                                     0.0, 1.0, NULL, NULL));
    CHECK(index == -1);
}

static void test_param_limits(void)
{
    char name[16];
    int index;
    int i;

    LOCKED(rb_init());
    for (i = 0; i < RB_MAX_PARAMS; ++i) {
        (void)snprintf(name, sizeof name, "p%02d", i);
        LOCKED(index = rb_param_register(name, "d", 0.0, 0.0, 1.0, NULL, NULL));
        CHECK(index == i);
    }
    LOCKED(index = rb_param_register("overflow", "d", 0.0, 0.0, 1.0, NULL,
                                     NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register("p00", "d", 0.0, 0.0, 1.0, NULL, NULL));
    CHECK(index == 0);
}

/* ------------------------------------------------------------ graph JSON */

static void test_graph_full(void)
{
    static char json[8192];
    char name[16];
    uint8_t payload[2] = {0u, 1u};
    size_t rc;
    size_t cap;
    int index;
    int i;

    LOCKED(rb_init());
    for (i = 0; i < RB_MAX_TOPICS; ++i) {
        (void)snprintf(name, sizeof name, "t%02d", i);
        LOCKED(index = rb_topic_register(name, "t"));
        CHECK(index == i);
        LOCKED((void)rb_publish(index, payload, sizeof payload));
    }
    for (i = 0; i < RB_MAX_SERVICES; ++i) {
        (void)snprintf(name, sizeof name, "svc%02d", i);
        LOCKED(index = rb_service_register(name, svc_fail, NULL));
        CHECK(index == i);
    }
    for (i = 0; i < RB_MAX_ACTIONS; ++i) {
        uint32_t goal_id;

        (void)snprintf(name, sizeof name, "act%02d", i);
        LOCKED(index = rb_action_register(name, step_fail, NULL));
        CHECK(index == i);
        LOCKED(goal_id = rb_action_send(name, "{}"));
        CHECK(goal_id == (uint32_t)(i + 1));
    }
    for (i = 0; i < RB_MAX_PARAMS; ++i) {
        (void)snprintf(name, sizeof name, "p%02d", i);
        LOCKED(index = rb_param_register(name, "d", 0.5, 0.0, 1.0, NULL,
                                         NULL));
        CHECK(index == i);
    }

    /* Everything registered: every element must be present in a big buffer. */
    memset(json, 0x5A, sizeof json);
    LOCKED(rc = rb_graph_json(json, sizeof json));
    CHECK(rc > 52u);
    CHECK(rc < sizeof json);
    CHECK(rc == strlen(json));
    CHECK(json[rc] == '\0');
    CHECK(json_valid(json));
    CHECK(count_substr(json, "\"name\":") ==
          RB_MAX_TOPICS + RB_MAX_SERVICES + RB_MAX_ACTIONS + RB_MAX_PARAMS);
    CHECK(strstr(json, "\"name\":\"t23\"") != NULL);
    CHECK(strstr(json, "\"name\":\"svc19\"") != NULL);
    CHECK(strstr(json, "\"name\":\"act09\"") != NULL);
    CHECK(strstr(json, "\"name\":\"p19\"") != NULL);
    CHECK(strstr(json, "\"state\":\"active\"") != NULL);

    /* Truncation sweep across the now-dense document: every capacity from the
     * bare skeleton upward must still yield valid JSON and an honest length. */
    for (cap = 53u; cap <= 400u; ++cap) {
        char small[420];

        memset(small, 0x5A, sizeof small);
        rc = rb_graph_json(small, cap);
        CHECK(rc == strlen(small));
        CHECK(rc <= cap - 1u);
        CHECK(small[rc] == '\0');
        CHECK(json_valid(small));
    }
}

static void test_graph_json(void)
{
    static char json[4096];
    char small[200];
    char tiny[4];
    const char skeleton[] =
        "{\"topics\":[],\"services\":[],\"actions\":[],\"params\":[]}";
    uint8_t payload[2] = {7u, 8u};
    uint8_t response[8];
    size_t response_len = sizeof response;
    sub_counter_t counter;
    rb_goal_t goal;
    uint32_t goal_id;
    size_t rc;
    size_t cap;
    int index;
    bool ok;

    memset(&counter, 0, sizeof counter);
    memset(&goal, 0, sizeof goal);
    LOCKED(rb_init());
    g_now = 1000u;

    /* Empty graph: exact skeleton and its documented length (52, no NUL). */
    LOCKED(rc = rb_graph_json(json, sizeof json));
    CHECK(rc == 52u);
    CHECK(rc == strlen(json));
    CHECK(json[rc] == '\0');
    CHECK(strcmp(json, skeleton) == 0);
    CHECK(sizeof skeleton == 53u); /* 52 content bytes + NUL */
    CHECK(json_valid(json));

    /* Populate every registry type, then check field names and counts. */
    LOCKED(index = rb_topic_register("cmd", "std_msgs/String"));
    CHECK(index == 0);
    LOCKED(ok = rb_publish(index, payload, sizeof payload));
    CHECK(ok);
    g_now = 1100u;
    LOCKED(ok = rb_publish(index, payload, sizeof payload));
    CHECK(ok);
    LOCKED(ok = rb_subscribe(index, sub_count_cb, &counter));
    CHECK(ok);
    LOCKED(index = rb_service_register("math/add", svc_echo, NULL));
    CHECK(index == 0);
    LOCKED(ok = rb_service_call("math/add", payload, sizeof payload, response,
                                &response_len));
    CHECK(ok);
    CHECK(response_len == sizeof payload);
    LOCKED(index = rb_action_register("goto", step_fail, NULL));
    CHECK(index == 0);
    LOCKED(goal_id = rb_action_send("goto", "{}"));
    CHECK(goal_id == 1u);
    LOCKED(index = rb_param_register("speed", "double", 30.0, 0.0, 100.0,
                                     NULL, NULL));
    CHECK(index == 0);
    LOCKED(index = rb_topic_register("q\"uote\\x", "t"));
    CHECK(index == 1);

    LOCKED(rc = rb_graph_json(json, sizeof json));
    CHECK(rc > 52u);
    CHECK(rc == strlen(json));
    CHECK(json[rc] == '\0');
    CHECK(json_valid(json));
    CHECK(strstr(json, "\"topics\"") != NULL);
    CHECK(strstr(json, "\"services\"") != NULL);
    CHECK(strstr(json, "\"actions\"") != NULL);
    CHECK(strstr(json, "\"params\"") != NULL);
    CHECK(strstr(json, "\"name\":\"cmd\"") != NULL);
    CHECK(strstr(json, "\"type\":\"std_msgs/String\"") != NULL);
    CHECK(strstr(json, "\"published\":2") != NULL);
    CHECK(strstr(json, "\"subs\":1") != NULL);
    CHECK(strstr(json, "\"hz\":10.0") != NULL);
    CHECK(strstr(json, "\"name\":\"math/add\"") != NULL);
    CHECK(strstr(json, "\"calls\":1") != NULL);
    CHECK(strstr(json, "\"errors\":0") != NULL);
    CHECK(strstr(json, "\"name\":\"goto\"") != NULL);
    CHECK(strstr(json, "\"sends\":1") != NULL);
    CHECK(strstr(json, "\"state\":\"active\"") != NULL);
    CHECK(strstr(json, "\"name\":\"speed\"") != NULL);
    CHECK(strstr(json, "\"value\":30") != NULL);
    CHECK(strstr(json, "\"min\":0") != NULL);
    CHECK(strstr(json, "\"max\":100") != NULL);
    /* Embedded quote/backslash in a name must be escaped, not copied raw. */
    CHECK(strstr(json, "\\\"") != NULL);
    CHECK(strstr(json, "\\\\") != NULL);

    /* Safe truncation: every capacity from the minimal skeleton upward must
     * still produce valid JSON, NUL termination and an honest return value. */
    for (cap = 53u; cap <= 200u; ++cap) {
        memset(small, 0x5A, sizeof small);
        rc = rb_graph_json(small, cap);
        CHECK(rc == strlen(small));
        CHECK(rc <= cap - 1u);
        CHECK(small[rc] == '\0');
        CHECK(json_valid(small));
    }
    /* A capacity of exactly 53 bytes fits only the skeleton. */
    memset(small, 0x5A, sizeof small);
    rc = rb_graph_json(small, sizeof skeleton);
    CHECK(rc == 52u);
    CHECK(strcmp(small, skeleton) == 0);
    LOCKED(rc = rb_graph_json(small, sizeof skeleton)); /* lock on this path */
    CHECK(rc == 52u);

    /* Not even the skeleton fits: empty string, 0 returned. */
    memset(tiny, 0x5A, sizeof tiny);
    LOCKED(rc = rb_graph_json(tiny, 52u));
    CHECK(rc == 0u);
    CHECK(tiny[0] == '\0');
    LOCKED(rc = rb_graph_json(tiny, 1u));
    CHECK(rc == 0u && tiny[0] == '\0');
    LOCKED(rc = rb_graph_json(tiny, 0u));
    CHECK(rc == 0u);
    LOCKED(rc = rb_graph_json(NULL, 100u));
    CHECK(rc == 0u);

    /* rb_init clears the model. */
    LOCKED(rb_init());
    LOCKED(rc = rb_graph_json(json, sizeof json));
    CHECK(rc == 52u);
    CHECK(strcmp(json, skeleton) == 0);
    CHECK(rb_topic_find("cmd") == -1);
}

/* ---------------------------------------------------------------- audit log */

static void fill_log(int count)
{
    int i;
    char event[16];
    char detail[16];

    for (i = 1; i <= count; ++i) {
        (void)snprintf(event, sizeof event, "e%02d", i);
        (void)snprintf(detail, sizeof detail, "d%02d", i);
        rb_log_event("src", event, detail);
    }
}

static void test_log_json(void)
{
    static char json[4096];
    char small[300];
    char tiny[8];
    char long_detail[200];
    const char *first;
    const char *p07;
    const char *p30;
    size_t rc;
    size_t cap;
    int i;

    LOCKED(rb_init());
    g_now = 1234u;

    /* Empty ring. */
    LOCKED(rc = rb_log_json(json, sizeof json));
    CHECK(rc == 14u);
    CHECK(rc == strlen(json));
    CHECK(strcmp(json, "{\"entries\":[]}") == 0);
    CHECK(json_valid(json));

    /* 30 events wrap the 24-entry ring; only e07..e30 survive, oldest first. */
    {
        unsigned long lock0 = g_lock_calls;
        unsigned long unlock0 = g_unlock_calls;

        fill_log(30); /* one public call per event */
        CHECK(g_lock_calls == lock0 + 30ul);
        CHECK(g_unlock_calls == unlock0 + 30ul);
        CHECK(g_lock_depth == 0);
    }
    LOCKED(rc = rb_log_json(json, sizeof json));
    CHECK(rc > 0u);
    CHECK(rc == strlen(json));
    CHECK(json[rc] == '\0');
    CHECK(json_valid(json));
    CHECK(count_substr(json, "\"source\"") == RB_MAX_LOG_ENTRIES);
    CHECK(strstr(json, "\"e07\"") != NULL);
    CHECK(strstr(json, "\"e30\"") != NULL);
    CHECK(strstr(json, "\"e06\"") == NULL);
    CHECK(strstr(json, "\"e01\"") == NULL);
    first = strstr(json, "\"event\":");
    CHECK(first != NULL);
    CHECK(first != NULL && strncmp(first + 8, "\"e07\"", 5) == 0);
    p07 = strstr(json, "\"e07\"");
    p30 = strstr(json, "\"e30\"");
    CHECK(p07 != NULL && p30 != NULL && p07 < p30);
    CHECK(strstr(json, "\"t\":1234") != NULL);
    CHECK(strstr(json, "\"source\":\"src\"") != NULL);
    CHECK(strstr(json, "\"detail\":\"d30\"") != NULL);

    /* Truncation keeps the newest entries (the ring is newest-biased). */
    memset(small, 0x5A, sizeof small);
    rc = rb_log_json(small, sizeof small);
    CHECK(rc == strlen(small));
    CHECK(rc <= sizeof small - 1u);
    CHECK(small[rc] == '\0');
    CHECK(json_valid(small));
    CHECK(strstr(small, "\"e30\"") != NULL);
    CHECK(strstr(small, "\"e07\"") == NULL);

    /* Every capacity from the minimal skeleton upward stays valid JSON. */
    for (cap = 15u; cap <= 120u; ++cap) {
        char buffer[130];

        memset(buffer, 0x5A, sizeof buffer);
        rc = rb_log_json(buffer, cap);
        CHECK(rc == strlen(buffer));
        CHECK(rc <= cap - 1u);
        CHECK(buffer[rc] == '\0');
        CHECK(json_valid(buffer));
    }

    /* Minimal skeleton: exactly 14 content bytes, then too-small refusal. */
    LOCKED(rc = rb_log_json(small, 15u));
    CHECK(rc == 14u);
    CHECK(strcmp(small, "{\"entries\":[]}") == 0);
    memset(tiny, 0x5A, sizeof tiny);
    LOCKED(rc = rb_log_json(tiny, 14u));
    CHECK(rc == 0u);
    CHECK(tiny[0] == '\0');
    LOCKED(rc = rb_log_json(tiny, 1u));
    CHECK(rc == 0u);
    LOCKED(rc = rb_log_json(tiny, 0u));
    CHECK(rc == 0u);
    LOCKED(rc = rb_log_json(NULL, 100u));
    CHECK(rc == 0u);

    /* NULL field pointers are treated as empty strings. */
    LOCKED(rb_log_event(NULL, NULL, NULL));
    LOCKED(rc = rb_log_json(json, sizeof json));
    CHECK(rc > 0u);
    CHECK(json_valid(json));

    /* Quoting/backslash escaping keeps the JSON valid. */
    LOCKED(rb_init());
    LOCKED(rb_log_event("s\"rc", "ev\\x", "de\"tail\\line"));
    LOCKED(rc = rb_log_json(json, sizeof json));
    CHECK(rc > 0u);
    CHECK(json_valid(json));
    CHECK(strstr(json, "\\\"") != NULL);
    CHECK(strstr(json, "\\\\") != NULL);

    /* Oversized fields are truncated; the document stays valid. */
    LOCKED(rb_init());
    for (i = 0; i < (int)sizeof long_detail - 1; ++i) {
        long_detail[i] = (char)('a' + (i % 26));
    }
    long_detail[sizeof long_detail - 1] = '\0';
    LOCKED(rb_log_event(long_detail, long_detail, long_detail));
    LOCKED(rc = rb_log_json(json, sizeof json));
    CHECK(rc > 0u);
    CHECK(json_valid(json));
    CHECK(rc < sizeof long_detail + 100u);
}

/* --------------------------------------------------------------- locking */

/* One non-nested call per public entry point: exactly one lock/unlock pair. */
static void test_locking(void)
{
    uint8_t payload[2] = {1u, 2u};
    uint8_t out[RB_MAX_PAYLOAD];
    rb_goal_t goal;
    size_t got = 0u;
    size_t resp_len = 0u;
    uint32_t goal_id;
    double value = 0.0;
    char json[512];
    int index;
    bool ok;
    const char *name;

    LOCKED(rb_init());
    CHECK(g_lock_calls == g_unlock_calls);
    CHECK(g_lock_depth == 0);

    /* topics */
    LOCKED(index = rb_topic_register("lk/topic", "t"));
    CHECK(index == 0);
    LOCKED(index = rb_topic_find("lk/topic"));
    CHECK(index == 0);
    LOCKED(name = rb_topic_name(0));
    CHECK(name != NULL);
    LOCKED(ok = rb_publish(0, payload, sizeof payload));
    CHECK(ok);
    LOCKED(ok = rb_latest(0, out, sizeof out, &got));
    CHECK(ok);
    LOCKED(ok = rb_subscribe(0, sub_count_cb, NULL));
    CHECK(ok);
    LOCKED(ok = rb_publish(-1, payload, sizeof payload));
    CHECK(!ok);

    /* services */
    LOCKED(index = rb_service_register("lk/svc", svc_echo, NULL));
    CHECK(index == 0);
    LOCKED(index = rb_service_find("lk/svc"));
    CHECK(index == 0);
    LOCKED(name = rb_service_name(0));
    CHECK(name != NULL);
    LOCKED(ok = rb_service_call("lk/svc", NULL, 0u, NULL, &resp_len));
    CHECK(ok);
    LOCKED(ok = rb_service_call("lk/missing", NULL, 0u, NULL, &resp_len));
    CHECK(!ok);

    /* actions */
    LOCKED(index = rb_action_register("lk/act", step_fail, NULL));
    CHECK(index == 0);
    LOCKED(goal_id = rb_action_send("lk/act", "{}"));
    CHECK(goal_id == 1u);
    LOCKED(ok = rb_action_status(goal_id, &goal));
    CHECK(ok);
    LOCKED(rb_action_tick(5000u));
    LOCKED(ok = rb_action_status(goal_id, &goal));
    CHECK(ok && goal.state == RB_ACTION_FAILED);
    LOCKED(goal_id = rb_action_send("lk/missing", NULL));
    CHECK(goal_id == 0u);
    LOCKED(ok = rb_action_cancel(9999u));
    CHECK(!ok);

    /* params */
    LOCKED(index = rb_param_register("lk/param", "d", 1.0, 0.0, 2.0, NULL,
                                     NULL));
    CHECK(index == 0);
    LOCKED(index = rb_param_find("lk/param"));
    CHECK(index == 0);
    LOCKED(name = rb_param_name(0));
    CHECK(name != NULL);
    LOCKED(ok = rb_param_set("lk/param", 1.5));
    CHECK(ok);
    LOCKED(ok = rb_param_get("lk/param", &value));
    CHECK(ok && value == 1.5);

    /* self model + audit */
    LOCKED(rb_graph_json(json, sizeof json));
    CHECK(json[0] == '{');
    LOCKED(rb_log_event("lk", "entry", "detail"));
    LOCKED(rb_log_json(json, sizeof json));
    CHECK(json[0] == '{');

    /* Error paths are locked too. */
    LOCKED(index = rb_topic_register(NULL, NULL));
    CHECK(index == -1);
    LOCKED(ok = rb_publish(999, NULL, 0u));
    CHECK(!ok);
    LOCKED(ok = rb_latest(999, out, sizeof out, &got));
    CHECK(!ok);
    LOCKED(ok = rb_subscribe(999, sub_count_cb, NULL));
    CHECK(!ok);
    LOCKED(index = rb_service_register(NULL, NULL, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_action_register(NULL, NULL, NULL));
    CHECK(index == -1);
    LOCKED(index = rb_param_register(NULL, NULL, 0.0, 0.0, 1.0, NULL, NULL));
    CHECK(index == -1);
    LOCKED(ok = rb_param_set(NULL, 0.0));
    CHECK(!ok);
    LOCKED(ok = rb_param_get(NULL, &value));
    CHECK(!ok);
    LOCKED(rb_graph_json(NULL, 0u));
    LOCKED(rb_log_json(NULL, 0u));
    LOCKED(rb_action_tick(6000u));

    CHECK(g_lock_calls == g_unlock_calls);
    CHECK(g_lock_depth == 0);
    CHECK(g_lock_calls > 0ul);
    CHECK(g_max_lock_depth >= 1);
}

/* ------------------------------------------------------------------- main */

int main(void)
{
    test_topics();
    test_topic_limits();
    test_services();
    test_service_limits();
    test_actions();
    test_action_limits();
    test_params();
    test_param_limits();
    test_graph_full();
    test_graph_json();
    test_log_json();
    test_locking();

    CHECK(g_lock_calls == g_unlock_calls);
    CHECK(g_lock_depth == 0);

    printf("checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}