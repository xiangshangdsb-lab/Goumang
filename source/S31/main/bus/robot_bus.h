#pragma once

/*
 * robot_bus: a small ROS2-like graph for the Goumang robot.
 *
 * It provides named topics with a latched last value, named services with
 * request/response, named actions with goal/feedback/cancel, and named
 * parameters with range checking and an apply hook.  Everything is
 * single-threaded from the caller's point of view; rb_port_lock()/unlock()
 * protect the internal tables.
 *
 * This module must stay host-testable: the platform port is only the lock and
 * the clock, and host tests provide their own.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RB_MAX_TOPICS      24
#define RB_MAX_SUBS        4
#define RB_MAX_SERVICES    20
#define RB_MAX_ACTIONS     10
#define RB_MAX_PARAMS      20
#define RB_MAX_PAYLOAD     96
#define RB_MAX_NAME        24
#define RB_MAX_LOG_ENTRIES 24

/* ------------------------------------------------------------------ port */
/*
 * Implemented by robot_bus_port.c on target, and by the host test file.
 *
 * Locking contract: every public function below takes rb_port_lock() on entry
 * and rb_port_unlock() on every exit path.  User callbacks (subscribers,
 * services, action steps, param apply hooks) run while the lock is held and
 * may call other bus functions, so the target port MUST provide a RECURSIVE
 * mutex.  The host port may be a counting no-op.
 */
void rb_port_lock(void);
void rb_port_unlock(void);
uint32_t rb_port_now_ms(void);

/* --------------------------------------------------------------- lifecycle */
void rb_init(void);

/* ---------------------------------------------------------------- topics */
/* Names must be 1..RB_MAX_NAME-1 bytes; NULL/empty/too-long names return -1.
 * Registering an existing name is idempotent and returns its index.  The
 * registry-full case returns -1. */
int rb_topic_register(const char *name, const char *type);
int rb_topic_find(const char *name);
const char *rb_topic_name(int topic);
/* Payload larger than RB_MAX_PAYLOAD is rejected.  A successful publish
 * updates the latched value and then invokes every subscriber synchronously
 * (under the bus lock). */
bool rb_publish(int topic, const void *data, size_t len);
/* Copy the latched value; returns false when nothing was published yet.
 * At most cap bytes are copied and *len (when non-NULL) receives the number
 * of bytes actually copied, i.e. min(stored_len, cap). */
bool rb_latest(int topic, void *out, size_t cap, size_t *len);
bool rb_latest_recent(int topic, void *out, size_t cap, size_t *len, uint32_t max_age_ms);
typedef void (*rb_sub_cb_t)(int topic, const void *data, size_t len, void *ctx);
/* At most RB_MAX_SUBS subscribers per topic; further calls return false. */
bool rb_subscribe(int topic, rb_sub_cb_t cb, void *ctx);

/* -------------------------------------------------------------- services */
/* Same naming/duplicate/full rules as topics.  rb_service_call() returns
 * false when the name is unknown or the handler reports an error.  On entry
 * *resp_len is the response capacity; on success it receives the length
 * actually returned by the handler. */
typedef bool (*rb_service_cb_t)(const void *req, size_t req_len,
                                void *resp, size_t *resp_len, void *ctx);
int rb_service_register(const char *name, rb_service_cb_t cb, void *ctx);
int rb_service_find(const char *name);
const char *rb_service_name(int index);
bool rb_service_call(const char *name, const void *req, size_t req_len,
                     void *resp, size_t *resp_len);

/* --------------------------------------------------------------- actions */
typedef enum {
    RB_ACTION_IDLE = 0,
    RB_ACTION_ACTIVE,
    RB_ACTION_SUCCEEDED,
    RB_ACTION_FAILED,
    RB_ACTION_CANCELED,
} rb_action_state_t;

typedef struct {
    uint32_t id;
    char name[RB_MAX_NAME];
    char args[64];
    rb_action_state_t state;
    uint8_t progress;
    uint32_t started_ms;
    uint32_t updated_ms;
    char feedback[48];
} rb_goal_t;

typedef rb_action_state_t (*rb_action_step_t)(rb_goal_t *goal, void *ctx);
/* Same naming/duplicate/full rules as topics.  rb_action_send() returns the
 * new goal id, or 0 when the action is unknown, unregistered or already has
 * an ACTIVE goal (one goal per action slot).  Goal ids start at 1 and are
 * never reused. */
int rb_action_register(const char *name, rb_action_step_t step, void *ctx);
uint32_t rb_action_send(const char *name, const char *args_json);
/* Only an ACTIVE goal can be canceled; unknown/finished goals return false. */
bool rb_action_cancel(uint32_t goal_id);
void rb_action_cancel_all(void);
/* Copy the latest goal snapshot; false when the id is unknown. */
bool rb_action_status(uint32_t goal_id, rb_goal_t *out);
/* Advance active goals; call periodically (e.g. every 50 ms).  now_ms is
 * supplied by the caller (used for started_ms/updated_ms bookkeeping). */
void rb_action_tick(uint32_t now_ms);

/* ---------------------------------------------------------------- params */
/*
 * Same naming/duplicate/full rules as topics.  min/max/default must be finite
 * and default must be inside [min, max], otherwise registration returns -1.
 * The apply hook (when non-NULL) is called once with the default during
 * registration; if it returns false the registration is rejected.  During
 * rb_param_set() the hook runs before the new value commits: a false return
 * rejects the change (out of range values are rejected without calling the
 * hook at all) and the old value is kept.
 */
typedef bool (*rb_param_apply_t)(double value, void *ctx);
int rb_param_register(const char *name, const char *type, double def,
                      double min_value, double max_value,
                      rb_param_apply_t apply, void *ctx);
bool rb_param_set(const char *name, double value);
bool rb_param_get(const char *name, double *out);
int rb_param_find(const char *name);
const char *rb_param_name(int index);

/* ------------------------------------------------------------ self model */
/*
 * JSON object: { "topics":[...], "services":[...], "actions":[...],
 *                "params":[...] } truncated into buf.
 *
 * Element schemas (counts/ranges come from the live registry):
 *   topic   {"name","type","published","subs","hz"}
 *   service {"name","calls","errors"}
 *   action  {"name","sends","state","progress"}
 *   param   {"name","type","value","min","max"}
 *
 * Return value: number of JSON bytes written, NOT counting the terminating
 * NUL (so ret == strlen(buf), and buf[ret] == '\0').  The output is always a
 * syntactically valid JSON document.  When cap is too small for everything,
 * only whole elements are emitted and the document is closed at the last
 * complete element.  If cap cannot hold even the empty skeleton (52 bytes of
 * content, i.e. 53 bytes incl. NUL), buf[0] is set to '\0' and 0 is returned.
 */
size_t rb_graph_json(char *buf, size_t cap);

/* Audit ring shared by agent/UI: one line per control decision.
 * RB_MAX_LOG_ENTRIES most recent events are kept. */
void rb_log_event(const char *source, const char *event, const char *detail);
/*
 * JSON: {"entries":[{"t","source","event","detail"},...]} oldest first.
 * Same return/truncation rules as rb_graph_json(); the empty skeleton is
 * 14 bytes of content ({"entries":[]}), i.e. 15 bytes incl. NUL.  When the
 * buffer is too small for all entries, the most recent entries are kept.
 */
size_t rb_log_json(char *buf, size_t cap);