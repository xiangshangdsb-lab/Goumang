/*
 * robot_bus.c - small ROS2-like graph for the Goumang robot.
 *
 * Design notes (keep in sync with robot_bus.h):
 *
 *  - Locking: every public entry point takes rb_port_lock() first and
 *    rb_port_unlock() last, including error paths.  User callbacks (topic
 *    subscribers, service handlers, action steps, param apply hooks) are
 *    invoked while the lock is held, because they are allowed to call back
 *    into the bus (e.g. an action step that reads rb_action_status() or sets
 *    a param).  The firmware port therefore uses a RECURSIVE mutex; the host
 *    port in tests/test_bus.c is a counting no-op.
 *
 *  - Names: valid names are 1..RB_MAX_NAME-1 bytes, NUL terminated.  An
 *    empty/NULL/too-long name is rejected with -1 by every register/find
 *    function, so truncation never happens for names.  Type strings and log
 *    fields are truncated on purpose (they are descriptive, not identity).
 *
 *  - Duplicate registration is idempotent: it returns the existing index and
 *    keeps the first registration's callbacks/metadata.  The registry-full
 *    check happens only for new names, so duplicates still work when full.
 *
 *  - rb_graph_json()/rb_log_json(): return the number of JSON bytes written,
 *    NOT counting the terminating NUL (i.e. return == strlen(buf) on success,
 *    and buf[return] == '\0').  The output is always a syntactically valid
 *    JSON document: when the buffer is too small, only whole elements are
 *    emitted and the document is closed at the last complete element.  If the
 *    buffer cannot even hold the empty skeleton (52 bytes for the graph,
 *    14 bytes for the log) an empty string is written and 0 is returned.
 *
 *  - rb_action_tick(now_ms): the caller supplies the timestamp, so host tests
 *    can drive time deterministically.  One goal per registered action; send
 *    while a goal is still ACTIVE returns 0 (goal ids are never reused).
 */

#include "robot_bus.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Internal log field limits (bytes incl. NUL).  Kept small on purpose. */
#define RB_LOG_SOURCE_MAX 16
#define RB_LOG_EVENT_MAX  24
#define RB_LOG_DETAIL_MAX 64

/* Exact byte counts of the graph JSON skeleton, used to guarantee that a
 * truncated document can still be closed into valid JSON. */
#define RB_GRAPH_PREFIX_TOPICS "{\"topics\":["
#define RB_GRAPH_PREFIX_SERVICES ",\"services\":["
#define RB_GRAPH_PREFIX_ACTIONS ",\"actions\":["
#define RB_GRAPH_PREFIX_PARAMS ",\"params\":["
#define RB_GRAPH_AFTER_TOPICS 41   /* ] + ",\"services\":[]" ... "}" */
#define RB_GRAPH_AFTER_SERVICES 27
#define RB_GRAPH_AFTER_ACTIONS 14
#define RB_GRAPH_AFTER_PARAMS 2
#define RB_GRAPH_MIN_BYTES 52      /* empty object, content bytes (no NUL) */

#define RB_LOG_PREFIX "{\"entries\":["
#define RB_LOG_SUFFIX "]}"         /* only the closing pair is fixed */
#define RB_LOG_MIN_BYTES 14        /* {"entries":[]} content bytes (no NUL) */

/* --------------------------------------------------------------- storage */

typedef struct {
    rb_sub_cb_t cb;
    void *ctx;
} rb_sub_slot_t;

typedef struct {
    bool used;
    char name[RB_MAX_NAME];
    char type[RB_MAX_NAME];
    uint8_t payload[RB_MAX_PAYLOAD];
    size_t payload_len;
    bool has_value;
    uint32_t publish_count;
    uint32_t last_publish_ms;
    uint32_t prev_publish_ms;
    uint8_t sub_count;
    rb_sub_slot_t subs[RB_MAX_SUBS];
} rb_topic_slot_t;

typedef struct {
    bool used;
    char name[RB_MAX_NAME];
    rb_service_cb_t cb;
    void *ctx;
    uint32_t calls;
    uint32_t errors;
} rb_service_slot_t;

typedef struct {
    bool used;
    char name[RB_MAX_NAME];
    rb_action_step_t step;
    void *ctx;
    uint32_t sends;
    rb_goal_t goal; /* goal.id == 0 means "no goal ever sent" */
} rb_action_slot_t;

typedef struct {
    bool used;
    char name[RB_MAX_NAME];
    char type[RB_MAX_NAME];
    double value;
    double default_value;
    double min_value;
    double max_value;
    rb_param_apply_t apply;
    void *ctx;
} rb_param_slot_t;

typedef struct {
    uint32_t t_ms;
    char source[RB_LOG_SOURCE_MAX];
    char event[RB_LOG_EVENT_MAX];
    char detail[RB_LOG_DETAIL_MAX];
} rb_log_entry_t;

static rb_topic_slot_t s_topics[RB_MAX_TOPICS];
static rb_service_slot_t s_services[RB_MAX_SERVICES];
static rb_action_slot_t s_actions[RB_MAX_ACTIONS];
static rb_param_slot_t s_params[RB_MAX_PARAMS];
static rb_log_entry_t s_log[RB_MAX_LOG_ENTRIES];
static uint32_t s_log_total; /* saturating total number of events logged */
static uint32_t s_log_write; /* next ring slot to write */
static uint32_t s_goal_next; /* last goal id handed out (0 = none yet) */

/* --------------------------------------------------------- small helpers */

static bool rb_name_ok(const char *name)
{
    size_t n;

    if (name == NULL || name[0] == '\0') {
        return false;
    }
    n = strlen(name);
    return n < (size_t)RB_MAX_NAME;
}

/* Copy a descriptive string, truncating at cap-1.  dst must have cap >= 1. */
static void rb_copy_into(char *dst, size_t cap, const char *src)
{
    size_t n;

    if (cap == 0) {
        return;
    }
    if (src == NULL) {
        dst[0] = '\0';
        return;
    }
    n = strlen(src);
    if (n >= cap) {
        n = cap - 1;
    }
    if (n > 0) {
        memcpy(dst, src, n);
    }
    dst[n] = '\0';
}

static int rb_topic_find_locked(const char *name)
{
    int i;

    if (name == NULL) {
        return -1;
    }
    for (i = 0; i < RB_MAX_TOPICS; ++i) {
        if (s_topics[i].used && strcmp(s_topics[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static int rb_service_find_locked(const char *name)
{
    int i;

    if (name == NULL) {
        return -1;
    }
    for (i = 0; i < RB_MAX_SERVICES; ++i) {
        if (s_services[i].used && strcmp(s_services[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static int rb_action_find_locked(const char *name)
{
    int i;

    if (name == NULL) {
        return -1;
    }
    for (i = 0; i < RB_MAX_ACTIONS; ++i) {
        if (s_actions[i].used && strcmp(s_actions[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static int rb_param_find_locked(const char *name)
{
    int i;

    if (name == NULL) {
        return -1;
    }
    for (i = 0; i < RB_MAX_PARAMS; ++i) {
        if (s_params[i].used && strcmp(s_params[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

static const char *rb_action_state_locked(rb_action_state_t state)
{
    switch (state) {
    case RB_ACTION_ACTIVE:
        return "active";
    case RB_ACTION_SUCCEEDED:
        return "succeeded";
    case RB_ACTION_FAILED:
        return "failed";
    case RB_ACTION_CANCELED:
        return "canceled";
    case RB_ACTION_IDLE:
    default:
        return "idle";
    }
}

/* ------------------------------------------------------------ JSON writer */

typedef struct {
    char *buf;
    size_t cap; /* max content bytes that fit, excluding the NUL */
    size_t len;
    bool overflow;
} rb_jw_t;

static void rb_jw_init(rb_jw_t *w, char *buf, size_t cap)
{
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->overflow = false;
}

static bool rb_jw_raw(rb_jw_t *w, const char *s)
{
    size_t n;

    if (w->overflow) {
        return false;
    }
    n = strlen(s);
    if (n > w->cap - w->len) {
        w->overflow = true;
        return false;
    }
    if (n > 0) {
        memcpy(w->buf + w->len, s, n);
    }
    w->len += n;
    return true;
}

static bool rb_jw_char(rb_jw_t *w, char c)
{
    if (w->overflow) {
        return false;
    }
    if (w->len >= w->cap) {
        w->overflow = true;
        return false;
    }
    w->buf[w->len] = c;
    ++w->len;
    return true;
}

static bool rb_jw_printf(rb_jw_t *w, const char *fmt, ...)
{
    char tmp[64];
    int n;
    va_list ap;

    if (w->overflow) {
        return false;
    }
    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sizeof tmp) {
        w->overflow = true;
        return false;
    }
    return rb_jw_raw(w, tmp);
}

static bool rb_jw_string(rb_jw_t *w, const char *s)
{
    const unsigned char *p = (const unsigned char *)s;

    if (!rb_jw_char(w, '"')) {
        return false;
    }
    if (p == NULL) {
        p = (const unsigned char *)"";
    }
    while (*p != 0) {
        unsigned char c = *p++;

        switch (c) {
        case '"':
            rb_jw_raw(w, "\\\"");
            break;
        case '\\':
            rb_jw_raw(w, "\\\\");
            break;
        case '\b':
            rb_jw_raw(w, "\\b");
            break;
        case '\f':
            rb_jw_raw(w, "\\f");
            break;
        case '\n':
            rb_jw_raw(w, "\\n");
            break;
        case '\r':
            rb_jw_raw(w, "\\r");
            break;
        case '\t':
            rb_jw_raw(w, "\\t");
            break;
        default:
            if (c < 0x20u) {
                rb_jw_printf(w, "\\u%04x", (unsigned)c);
            } else {
                rb_jw_char(w, (char)c);
            }
            break;
        }
        if (w->overflow) {
            return false;
        }
    }
    return rb_jw_char(w, '"');
}

/* Copy a fixed-size buffer field as a JSON string.  The source is truncated
 * to `cap-1` bytes so the result is always NUL terminated. */
static bool rb_jw_fixed_string(rb_jw_t *w, const char *src, size_t cap)
{
    char tmp[RB_LOG_DETAIL_MAX];
    size_t n;

    if (cap > sizeof tmp) {
        cap = sizeof tmp;
    }
    if (cap == 0) {
        return rb_jw_string(w, "");
    }
    n = strlen(src);
    if (n >= cap) {
        n = cap - 1;
    }
    memcpy(tmp, src, n);
    tmp[n] = '\0';
    return rb_jw_string(w, tmp);
}

/* --------------------------------------------------------- JSON fragments */

/* One graph element per registry entry.  `first` controls the leading comma.
 * Returns the produced length (bytes excluding NUL) or 0 on overflow. */
static size_t rb_topic_json_elem(char *dst, size_t dst_size,
                                 const rb_topic_slot_t *t, bool first)
{
    rb_jw_t w;
    unsigned long hz10 = 0;
    uint32_t dt;

    rb_jw_init(&w, dst, dst_size - 1);
    if (!first) {
        rb_jw_raw(&w, ",");
    }
    rb_jw_raw(&w, "{\"name\":");
    rb_jw_string(&w, t->name);
    rb_jw_raw(&w, ",\"type\":");
    rb_jw_string(&w, t->type);
    dt = t->last_publish_ms - t->prev_publish_ms;
    if (t->publish_count >= 2u && dt > 0u) {
        hz10 = 10000UL / (unsigned long)dt;
        if (hz10 > 999999UL) {
            hz10 = 999999UL;
        }
    }
    rb_jw_printf(&w, ",\"published\":%lu,\"subs\":%u,\"hz\":%lu.%01lu",
                 (unsigned long)t->publish_count, (unsigned)t->sub_count,
                 hz10 / 10UL, hz10 % 10UL);
    rb_jw_raw(&w, "}");
    if (w.overflow) {
        return 0;
    }
    dst[w.len] = '\0';
    return w.len;
}

static size_t rb_service_json_elem(char *dst, size_t dst_size,
                                   const rb_service_slot_t *s, bool first)
{
    rb_jw_t w;

    rb_jw_init(&w, dst, dst_size - 1);
    if (!first) {
        rb_jw_raw(&w, ",");
    }
    rb_jw_raw(&w, "{\"name\":");
    rb_jw_string(&w, s->name);
    rb_jw_printf(&w, ",\"calls\":%lu,\"errors\":%lu",
                 (unsigned long)s->calls, (unsigned long)s->errors);
    rb_jw_raw(&w, "}");
    if (w.overflow) {
        return 0;
    }
    dst[w.len] = '\0';
    return w.len;
}

static size_t rb_action_json_elem(char *dst, size_t dst_size,
                                  const rb_action_slot_t *a, bool first)
{
    rb_jw_t w;
    rb_action_state_t state;
    uint8_t progress;

    state = (a->goal.id != 0u) ? a->goal.state : RB_ACTION_IDLE;
    progress = (a->goal.id != 0u) ? a->goal.progress : 0u;
    rb_jw_init(&w, dst, dst_size - 1);
    if (!first) {
        rb_jw_raw(&w, ",");
    }
    rb_jw_raw(&w, "{\"name\":");
    rb_jw_string(&w, a->name);
    rb_jw_printf(&w, ",\"sends\":%lu,\"state\":\"%s\",\"progress\":%u",
                 (unsigned long)a->sends, rb_action_state_locked(state),
                 (unsigned)progress);
    rb_jw_raw(&w, "}");
    if (w.overflow) {
        return 0;
    }
    dst[w.len] = '\0';
    return w.len;
}

static size_t rb_param_json_elem(char *dst, size_t dst_size,
                                 const rb_param_slot_t *p, bool first)
{
    rb_jw_t w;

    rb_jw_init(&w, dst, dst_size - 1);
    if (!first) {
        rb_jw_raw(&w, ",");
    }
    rb_jw_raw(&w, "{\"name\":");
    rb_jw_string(&w, p->name);
    rb_jw_raw(&w, ",\"type\":");
    rb_jw_string(&w, p->type);
    rb_jw_printf(&w, ",\"value\":%g,\"min\":%g,\"max\":%g",
                 p->value, p->min_value, p->max_value);
    rb_jw_raw(&w, "}");
    if (w.overflow) {
        return 0;
    }
    dst[w.len] = '\0';
    return w.len;
}

static size_t rb_log_json_elem(char *dst, size_t dst_size,
                               const rb_log_entry_t *e)
{
    rb_jw_t w;

    rb_jw_init(&w, dst, dst_size - 1);
    rb_jw_printf(&w, "{\"t\":%lu,\"source\":", (unsigned long)e->t_ms);
    rb_jw_fixed_string(&w, e->source, RB_LOG_SOURCE_MAX);
    rb_jw_raw(&w, ",\"event\":");
    rb_jw_fixed_string(&w, e->event, RB_LOG_EVENT_MAX);
    rb_jw_raw(&w, ",\"detail\":");
    rb_jw_fixed_string(&w, e->detail, RB_LOG_DETAIL_MAX);
    rb_jw_raw(&w, "}");
    if (w.overflow) {
        return 0;
    }
    dst[w.len] = '\0';
    return w.len;
}

static void rb_json_put(char *buf, size_t *len, const char *s)
{
    size_t n = strlen(s);

    if (n > 0) {
        memcpy(buf + *len, s, n);
    }
    *len += n;
}

/* --------------------------------------------------------------- lifecycle */

void rb_init(void)
{
    rb_port_lock();
    memset(s_topics, 0, sizeof s_topics);
    memset(s_services, 0, sizeof s_services);
    memset(s_actions, 0, sizeof s_actions);
    memset(s_params, 0, sizeof s_params);
    memset(s_log, 0, sizeof s_log);
    s_log_total = 0;
    s_log_write = 0;
    s_goal_next = 0;
    rb_port_unlock();
}

/* ------------------------------------------------------------------ topics */

int rb_topic_register(const char *name, const char *type)
{
    int i;
    int slot = -1;
    int existing;

    rb_port_lock();
    if (!rb_name_ok(name)) {
        rb_port_unlock();
        return -1;
    }
    existing = rb_topic_find_locked(name);
    if (existing >= 0) {
        rb_port_unlock();
        return existing;
    }
    for (i = 0; i < RB_MAX_TOPICS; ++i) {
        if (!s_topics[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        rb_port_unlock();
        return -1;
    }
    memset(&s_topics[slot], 0, sizeof s_topics[slot]);
    s_topics[slot].used = true;
    rb_copy_into(s_topics[slot].name, sizeof s_topics[slot].name, name);
    rb_copy_into(s_topics[slot].type, sizeof s_topics[slot].type, type);
    rb_port_unlock();
    return slot;
}

int rb_topic_find(const char *name)
{
    int index;

    rb_port_lock();
    index = rb_topic_find_locked(name);
    rb_port_unlock();
    return index;
}

const char *rb_topic_name(int topic)
{
    const char *name = NULL;

    rb_port_lock();
    if (topic >= 0 && topic < RB_MAX_TOPICS && s_topics[topic].used) {
        name = s_topics[topic].name;
    }
    rb_port_unlock();
    return name;
}

bool rb_publish(int topic, const void *data, size_t len)
{
    uint8_t snapshot[RB_MAX_PAYLOAD];
    bool ok = false;
    size_t i;

    rb_port_lock();
    if (topic >= 0 && topic < RB_MAX_TOPICS && s_topics[topic].used &&
        len <= (size_t)RB_MAX_PAYLOAD && (data != NULL || len == 0u)) {
        rb_topic_slot_t *t = &s_topics[topic];

        if (len > 0u) {
            memcpy(t->payload, data, len);
        }
        t->payload_len = len;
        t->has_value = true;
        t->publish_count += 1u;
        t->prev_publish_ms = t->last_publish_ms;
        t->last_publish_ms = rb_port_now_ms();
        /* Callbacks get a private snapshot, so a subscriber that publishes
         * again cannot pull the payload out from under its siblings. */
        if (len > 0u) {
            memcpy(snapshot, t->payload, len);
        }
        for (i = 0; i < (size_t)t->sub_count; ++i) {
            if (t->subs[i].cb != NULL) {
                t->subs[i].cb(topic, snapshot, len, t->subs[i].ctx);
            }
        }
        ok = true;
    }
    rb_port_unlock();
    return ok;
}

static bool latest_locked(int topic, void *out, size_t cap, size_t *len,
                           uint32_t max_age_ms)
{
    bool ok = false;

    if (topic >= 0 && topic < RB_MAX_TOPICS && s_topics[topic].used &&
        s_topics[topic].has_value && out != NULL &&
        (max_age_ms == UINT32_MAX ||
         rb_port_now_ms() - s_topics[topic].last_publish_ms <= max_age_ms)) {
        size_t n = s_topics[topic].payload_len;

        if (n > cap) {
            n = cap;
        }
        if (n > 0u) {
            memcpy(out, s_topics[topic].payload, n);
        }
        if (len != NULL) {
            *len = n;
        }
        ok = true;
    }
    return ok;
}

bool rb_latest_recent(int topic, void *out, size_t cap, size_t *len, uint32_t max_age_ms)
{
    rb_port_lock();
    bool ok = latest_locked(topic, out, cap, len, max_age_ms);
    rb_port_unlock();
    return ok;
}

bool rb_latest(int topic, void *out, size_t cap, size_t *len)
{
    return rb_latest_recent(topic, out, cap, len, UINT32_MAX);
}

bool rb_subscribe(int topic, rb_sub_cb_t cb, void *ctx)
{
    bool ok = false;

    rb_port_lock();
    if (topic >= 0 && topic < RB_MAX_TOPICS && s_topics[topic].used &&
        cb != NULL && s_topics[topic].sub_count < (uint8_t)RB_MAX_SUBS) {
        rb_topic_slot_t *t = &s_topics[topic];

        t->subs[t->sub_count].cb = cb;
        t->subs[t->sub_count].ctx = ctx;
        t->sub_count = (uint8_t)(t->sub_count + 1u);
        ok = true;
    }
    rb_port_unlock();
    return ok;
}

/* ---------------------------------------------------------------- services */

int rb_service_register(const char *name, rb_service_cb_t cb, void *ctx)
{
    int i;
    int slot = -1;
    int existing;

    rb_port_lock();
    if (!rb_name_ok(name) || cb == NULL) {
        rb_port_unlock();
        return -1;
    }
    existing = rb_service_find_locked(name);
    if (existing >= 0) {
        rb_port_unlock();
        return existing;
    }
    for (i = 0; i < RB_MAX_SERVICES; ++i) {
        if (!s_services[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        rb_port_unlock();
        return -1;
    }
    memset(&s_services[slot], 0, sizeof s_services[slot]);
    s_services[slot].used = true;
    rb_copy_into(s_services[slot].name, sizeof s_services[slot].name, name);
    s_services[slot].cb = cb;
    s_services[slot].ctx = ctx;
    rb_port_unlock();
    return slot;
}

int rb_service_find(const char *name)
{
    int index;

    rb_port_lock();
    index = rb_service_find_locked(name);
    rb_port_unlock();
    return index;
}

const char *rb_service_name(int index)
{
    const char *name = NULL;

    rb_port_lock();
    if (index >= 0 && index < RB_MAX_SERVICES && s_services[index].used) {
        name = s_services[index].name;
    }
    rb_port_unlock();
    return name;
}

bool rb_service_call(const char *name, const void *req, size_t req_len,
                     void *resp, size_t *resp_len)
{
    bool ok = false;
    int index;
    size_t local_len;

    rb_port_lock();
    index = rb_service_find_locked(name);
    if (index >= 0 && s_services[index].cb != NULL) {
        rb_service_slot_t *s = &s_services[index];

        local_len = (resp_len != NULL) ? *resp_len : 0u;
        s->calls += 1u;
        ok = s->cb(req, req_len, resp, &local_len, s->ctx);
        if (!ok) {
            s->errors += 1u;
        }
        if (resp_len != NULL) {
            *resp_len = local_len;
        }
    }
    rb_port_unlock();
    return ok;
}

/* ----------------------------------------------------------------- actions */

int rb_action_register(const char *name, rb_action_step_t step, void *ctx)
{
    int i;
    int slot = -1;
    int existing;

    rb_port_lock();
    if (!rb_name_ok(name) || step == NULL) {
        rb_port_unlock();
        return -1;
    }
    existing = rb_action_find_locked(name);
    if (existing >= 0) {
        rb_port_unlock();
        return existing;
    }
    for (i = 0; i < RB_MAX_ACTIONS; ++i) {
        if (!s_actions[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        rb_port_unlock();
        return -1;
    }
    memset(&s_actions[slot], 0, sizeof s_actions[slot]);
    s_actions[slot].used = true;
    rb_copy_into(s_actions[slot].name, sizeof s_actions[slot].name, name);
    s_actions[slot].step = step;
    s_actions[slot].ctx = ctx;
    rb_port_unlock();
    return slot;
}

uint32_t rb_action_send(const char *name, const char *args_json)
{
    uint32_t goal_id = 0;
    int index;

    rb_port_lock();
    index = rb_action_find_locked(name);
    if (index >= 0) {
        rb_action_slot_t *a = &s_actions[index];

        if (a->goal.state != RB_ACTION_ACTIVE) {
            ++s_goal_next;
            if (s_goal_next == 0u) {
                ++s_goal_next; /* 0 is the "invalid goal" sentinel */
            }
            memset(&a->goal, 0, sizeof a->goal);
            a->goal.id = s_goal_next;
            rb_copy_into(a->goal.name, sizeof a->goal.name, a->name);
            rb_copy_into(a->goal.args, sizeof a->goal.args, args_json);
            a->goal.state = RB_ACTION_ACTIVE;
            a->goal.started_ms = rb_port_now_ms();
            a->goal.updated_ms = a->goal.started_ms;
            a->sends += 1u;
            goal_id = a->goal.id;
        }
    }
    rb_port_unlock();
    return goal_id;
}

bool rb_action_cancel(uint32_t goal_id)
{
    bool ok = false;
    int i;

    rb_port_lock();
    if (goal_id != 0u) {
        for (i = 0; i < RB_MAX_ACTIONS; ++i) {
            rb_action_slot_t *a = &s_actions[i];

            if (a->used && a->goal.id == goal_id) {
                if (a->goal.state == RB_ACTION_ACTIVE) {
                    a->goal.state = RB_ACTION_CANCELED;
                    a->goal.updated_ms = rb_port_now_ms();
                    ok = true;
                }
                break;
            }
        }
    }
    rb_port_unlock();
    return ok;
}

void rb_action_cancel_all(void)
{
    rb_port_lock();
    uint32_t now = rb_port_now_ms();
    for (int i = 0; i < RB_MAX_ACTIONS; ++i) {
        rb_action_slot_t *slot = &s_actions[i];
        if (slot->used && slot->goal.state == RB_ACTION_ACTIVE) {
            slot->goal.state = RB_ACTION_CANCELED;
            slot->goal.updated_ms = now;
        }
    }
    rb_port_unlock();
}

bool rb_action_status(uint32_t goal_id, rb_goal_t *out)
{
    bool ok = false;
    int i;

    rb_port_lock();
    if (goal_id != 0u && out != NULL) {
        for (i = 0; i < RB_MAX_ACTIONS; ++i) {
            if (s_actions[i].used && s_actions[i].goal.id == goal_id) {
                *out = s_actions[i].goal;
                ok = true;
                break;
            }
        }
    }
    rb_port_unlock();
    return ok;
}

void rb_action_tick(uint32_t now_ms)
{
    int i;

    rb_port_lock();
    for (i = 0; i < RB_MAX_ACTIONS; ++i) {
        rb_action_slot_t *a = &s_actions[i];
        rb_action_state_t next;

        if (!a->used || a->goal.id == 0u) {
            continue;
        }
        if (a->goal.state != RB_ACTION_ACTIVE) {
            continue;
        }
        if (a->step == NULL) {
            a->goal.state = RB_ACTION_FAILED;
            a->goal.updated_ms = now_ms;
            continue;
        }
        next = a->step(&a->goal, a->ctx);
        a->goal.updated_ms = now_ms;
        /* The step may have canceled itself through rb_action_cancel(); a
         * terminal state wins over an RB_ACTION_ACTIVE return value. */
        if (a->goal.state == RB_ACTION_ACTIVE) {
            if (next == RB_ACTION_SUCCEEDED || next == RB_ACTION_FAILED ||
                next == RB_ACTION_CANCELED) {
                a->goal.state = next;
            }
        }
    }
    rb_port_unlock();
}

/* ------------------------------------------------------------------ params */

int rb_param_register(const char *name, const char *type, double def,
                      double min_value, double max_value,
                      rb_param_apply_t apply, void *ctx)
{
    int i;
    int slot = -1;
    int existing;
    rb_param_slot_t candidate;

    rb_port_lock();
    if (!rb_name_ok(name)) {
        rb_port_unlock();
        return -1;
    }
    existing = rb_param_find_locked(name);
    if (existing >= 0) {
        rb_port_unlock();
        return existing;
    }
    if (!isfinite(def) || !isfinite(min_value) || !isfinite(max_value) ||
        !(min_value <= max_value) || !(def >= min_value && def <= max_value)) {
        rb_port_unlock();
        return -1;
    }
    for (i = 0; i < RB_MAX_PARAMS; ++i) {
        if (!s_params[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        rb_port_unlock();
        return -1;
    }
    memset(&candidate, 0, sizeof candidate);
    candidate.used = true;
    rb_copy_into(candidate.name, sizeof candidate.name, name);
    rb_copy_into(candidate.type, sizeof candidate.type, type);
    candidate.value = def;
    candidate.default_value = def;
    candidate.min_value = min_value;
    candidate.max_value = max_value;
    candidate.apply = apply;
    candidate.ctx = ctx;
    /* The apply hook is invoked while the lock is held; it may re-enter the
     * bus.  If it rejects the default, registration fails cleanly. */
    if (candidate.apply != NULL && !candidate.apply(candidate.value, candidate.ctx)) {
        rb_port_unlock();
        return -1;
    }
    s_params[slot] = candidate;
    rb_port_unlock();
    return slot;
}

bool rb_param_set(const char *name, double value)
{
    bool ok = false;
    int index;

    rb_port_lock();
    index = rb_param_find_locked(name);
    if (index >= 0 && isfinite(value)) {
        rb_param_slot_t *p = &s_params[index];

        if (value >= p->min_value && value <= p->max_value) {
            if (p->apply == NULL) {
                p->value = value;
                ok = true;
            } else {
                double previous = p->value;

                p->value = value;
                if (p->apply(value, p->ctx)) {
                    ok = true;
                } else {
                    p->value = previous;
                }
            }
        }
    }
    rb_port_unlock();
    return ok;
}

bool rb_param_get(const char *name, double *out)
{
    bool ok = false;
    int index;

    rb_port_lock();
    index = rb_param_find_locked(name);
    if (index >= 0 && out != NULL) {
        *out = s_params[index].value;
        ok = true;
    }
    rb_port_unlock();
    return ok;
}

int rb_param_find(const char *name)
{
    int index;

    rb_port_lock();
    index = rb_param_find_locked(name);
    rb_port_unlock();
    return index;
}

const char *rb_param_name(int index)
{
    const char *name = NULL;

    rb_port_lock();
    if (index >= 0 && index < RB_MAX_PARAMS && s_params[index].used) {
        name = s_params[index].name;
    }
    rb_port_unlock();
    return name;
}

/* ------------------------------------------------------------- self model */

/* Append one array section.  Callers pass the element builder and the exact
 * number of bytes needed to close this array plus the remaining empty
 * sections; an element is only emitted when it and that closing budget fit. */
typedef size_t (*rb_json_elem_fn)(char *dst, size_t dst_size,
                                  const void *entry, bool first);

/* Every slot struct starts with `bool used`; that is the only assumption this
 * stride-based walker makes.  A section stops adding elements as soon as the
 * next element plus the bytes needed to close the document no longer fit. */
static void rb_graph_section(char *buf, size_t *len, size_t limit,
                             const void *base, size_t stride, size_t count,
                             rb_json_elem_fn elem_fn, size_t after_bytes)
{
    char elem[512];
    bool first = true;
    size_t i;

    for (i = 0; i < count; ++i) {
        const unsigned char *p = (const unsigned char *)base + i * stride;
        size_t elem_len;

        if (!*(const bool *)p) {
            continue;
        }
        elem_len = elem_fn(elem, sizeof elem, p, first);
        if (elem_len == 0u) {
            continue; /* element alone is absurdly long; keep the rest valid */
        }
        if (*len + elem_len + after_bytes > limit) {
            break;
        }
        memcpy(buf + *len, elem, elem_len);
        *len += elem_len;
        first = false;
    }
}

static size_t rb_topic_elem_adapter(char *dst, size_t dst_size,
                                    const void *entry, bool first)
{
    return rb_topic_json_elem(dst, dst_size, (const rb_topic_slot_t *)entry,
                              first);
}

static size_t rb_service_elem_adapter(char *dst, size_t dst_size,
                                      const void *entry, bool first)
{
    return rb_service_json_elem(dst, dst_size,
                                (const rb_service_slot_t *)entry, first);
}

static size_t rb_action_elem_adapter(char *dst, size_t dst_size,
                                     const void *entry, bool first)
{
    return rb_action_json_elem(dst, dst_size, (const rb_action_slot_t *)entry,
                               first);
}

static size_t rb_param_elem_adapter(char *dst, size_t dst_size,
                                    const void *entry, bool first)
{
    return rb_param_json_elem(dst, dst_size, (const rb_param_slot_t *)entry,
                              first);
}

size_t rb_graph_json(char *buf, size_t cap)
{
    size_t len = 0;
    size_t limit;

    rb_port_lock();
    if (buf == NULL || cap == 0u) {
        rb_port_unlock();
        return 0;
    }
    limit = cap - 1u;
    if (limit < (size_t)RB_GRAPH_MIN_BYTES) {
        buf[0] = '\0';
        rb_port_unlock();
        return 0;
    }
    rb_json_put(buf, &len, RB_GRAPH_PREFIX_TOPICS);
    rb_graph_section(buf, &len, limit, s_topics, sizeof s_topics[0],
                     (size_t)RB_MAX_TOPICS, rb_topic_elem_adapter,
                     (size_t)RB_GRAPH_AFTER_TOPICS);
    rb_json_put(buf, &len, "]");
    rb_json_put(buf, &len, RB_GRAPH_PREFIX_SERVICES);
    rb_graph_section(buf, &len, limit, s_services, sizeof s_services[0],
                     (size_t)RB_MAX_SERVICES, rb_service_elem_adapter,
                     (size_t)RB_GRAPH_AFTER_SERVICES);
    rb_json_put(buf, &len, "]");
    rb_json_put(buf, &len, RB_GRAPH_PREFIX_ACTIONS);
    rb_graph_section(buf, &len, limit, s_actions, sizeof s_actions[0],
                     (size_t)RB_MAX_ACTIONS, rb_action_elem_adapter,
                     (size_t)RB_GRAPH_AFTER_ACTIONS);
    rb_json_put(buf, &len, "]");
    rb_json_put(buf, &len, RB_GRAPH_PREFIX_PARAMS);
    rb_graph_section(buf, &len, limit, s_params, sizeof s_params[0],
                     (size_t)RB_MAX_PARAMS, rb_param_elem_adapter,
                     (size_t)RB_GRAPH_AFTER_PARAMS);
    rb_json_put(buf, &len, "]}");
    buf[len] = '\0';
    rb_port_unlock();
    return len;
}

/* --------------------------------------------------------------------- log */

void rb_log_event(const char *source, const char *event, const char *detail)
{
    rb_log_entry_t *entry;

    rb_port_lock();
    entry = &s_log[s_log_write];
    memset(entry, 0, sizeof *entry);
    entry->t_ms = rb_port_now_ms();
    rb_copy_into(entry->source, sizeof entry->source,
                 (source != NULL) ? source : "");
    rb_copy_into(entry->event, sizeof entry->event,
                 (event != NULL) ? event : "");
    rb_copy_into(entry->detail, sizeof entry->detail,
                 (detail != NULL) ? detail : "");
    s_log_write = (s_log_write + 1u) % (uint32_t)RB_MAX_LOG_ENTRIES;
    if (s_log_total < 0xFFFFFFFFu) {
        s_log_total += 1u;
    }
    rb_port_unlock();
}

size_t rb_log_json(char *buf, size_t cap)
{
    size_t len = 0;
    size_t limit;
    char elem[256];
    uint32_t count;
    uint32_t keep = 0;
    uint32_t k;
    size_t total = 0;
    bool first = true;

    rb_port_lock();
    if (buf == NULL || cap == 0u) {
        rb_port_unlock();
        return 0;
    }
    limit = cap - 1u;
    if (limit < (size_t)RB_LOG_MIN_BYTES) {
        buf[0] = '\0';
        rb_port_unlock();
        return 0;
    }
    count = (s_log_total < (uint32_t)RB_MAX_LOG_ENTRIES)
                ? s_log_total
                : (uint32_t)RB_MAX_LOG_ENTRIES;
    /* First pass: find out how many of the newest entries fit, so a short
     * buffer keeps the most recent audit trail rather than the oldest.
     * RB_LOG_MIN_BYTES covers both the opening prefix and the closing "]}", so
     * `total` only has to account for the elements and their commas. */
    for (k = 0; k < count; ++k) {
        uint32_t index =
            (s_log_write + (uint32_t)RB_MAX_LOG_ENTRIES - 1u - k) %
            (uint32_t)RB_MAX_LOG_ENTRIES;
        size_t elem_len = rb_log_json_elem(elem, sizeof elem, &s_log[index]);
        size_t add = elem_len + ((keep > 0u) ? 1u : 0u);

        if (elem_len == 0u ||
            (size_t)RB_LOG_MIN_BYTES + total + add > limit) {
            break;
        }
        total += add;
        ++keep;
    }
    rb_json_put(buf, &len, RB_LOG_PREFIX);
    for (k = keep; k > 0u; --k) {
        uint32_t index = (s_log_write + (uint32_t)RB_MAX_LOG_ENTRIES - k) %
                         (uint32_t)RB_MAX_LOG_ENTRIES;
        size_t elem_len = rb_log_json_elem(elem, sizeof elem, &s_log[index]);

        if (!first) {
            buf[len] = ',';
            ++len;
        }
        memcpy(buf + len, elem, elem_len);
        len += elem_len;
        first = false;
    }
    rb_json_put(buf, &len, RB_LOG_SUFFIX);
    buf[len] = '\0';
    rb_port_unlock();
    return len;
}