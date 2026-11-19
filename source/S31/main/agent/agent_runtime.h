#pragma once

/*
 * agent_runtime: the on-device half of the AI agent.
 *
 * It owns the takeover mode, the control lease, the per-request action
 * budget, the local multi-step plan executor and the reply/audit buffers
 * that the screen and the web console display.
 *
 * The cloud model (agent_service) can only *ask*; this module decides what is
 * allowed.  Takeover can be enabled only by a human (screen or web with the
 * control code).  The model can renew the lease but never open the mode.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AGENT_MAX_USER      128
#define AGENT_MAX_REPLY     512
#define AGENT_MAX_PLAN      32
#define AGENT_MAX_PLAN_MS   120000u
#define AGENT_MAX_LOG       32

typedef enum {
    AGENT_MODE_ASSIST = 0,
    AGENT_MODE_TAKEOVER = 1,
} agent_mode_t;

typedef struct {
    agent_mode_t mode;
    bool takeover;
    uint32_t lease_remaining_ms;
    char last_user[AGENT_MAX_USER];
    char last_reply[AGENT_MAX_REPLY];
    char last_tool[32];
    char last_action[40];
    uint8_t plan_total;
    uint8_t plan_done;
    bool plan_running;
    uint32_t plan_id;
    char plan_error[64];
    uint32_t blocked_actions;
    uint32_t actions_total;
} agent_state_t;

bool agent_runtime_init(void);
void agent_runtime_tick(uint32_t now_ms);

/* ---- human authority ------------------------------------------------ */
/* Enable/disable takeover.  Enabling also starts a lease of lease_seconds.
 * Disabling stops motion immediately and returns to ASSIST. */
bool agent_runtime_set_takeover(bool on, uint32_t lease_seconds);
/* Renew the lease (AI heartbeat, or web keepalive).  Only valid in takeover. */
bool agent_runtime_lease(uint32_t seconds);
void agent_runtime_revoke(const char *reason);
void agent_runtime_abort_motion(const char *reason);
agent_mode_t agent_runtime_mode(void);
uint32_t agent_runtime_lease_remaining_ms(void);
/* Mode allows control AND lease is valid AND no e-stop/fault blocks it. */
bool agent_runtime_takeover_ok(void);

/* ---- AI-facing policy ----------------------------------------------- */
/* Begin a new chat request: resets the one-action budget used in ASSIST. */
void agent_runtime_begin_request(void);
/* Ask permission for one control action.  In ASSIST the first action of a
 * request is allowed; in TAKEOVER a valid lease is required. */
bool agent_runtime_allow_control(const char *tool);
/* Plan execution (TAKEOVER only).  Returns false with a message in err. */
bool agent_runtime_submit_plan(const char *json, char *err, size_t err_cap,
                               uint32_t *plan_id);
bool agent_runtime_cancel_plan(uint32_t plan_id);
bool agent_runtime_plan_snapshot(uint32_t *plan_id, uint8_t *total,
                                 uint8_t *done, bool *running,
                                 char *err, size_t err_cap);
/* Display buffers. */
void agent_runtime_set_reply(const char *user, const char *reply,
                             const char *tool);
void agent_runtime_set_action(const char *action);
void agent_runtime_state_snapshot(agent_state_t *out);
size_t agent_runtime_state_json(char *buf, size_t cap);
size_t agent_runtime_log_json(char *buf, size_t cap);
void agent_runtime_audit(const char *source, const char *event,
                         const char *detail);

/* ---- plan steps ------------------------------------------------------ */
typedef enum {
    AGENT_STEP_PAN = 0,
    AGENT_STEP_DRIVE,
    AGENT_STEP_LINE,
    AGENT_STEP_WAIT,
    AGENT_STEP_WAIT_LINE,
    AGENT_STEP_PARAM,
    AGENT_STEP_CAPTURE,
    AGENT_STEP_SAY,
    AGENT_STEP_STOP,
} agent_step_op_t;

typedef struct {
    agent_step_op_t op;
    int32_t a;             /* pan deg / left / speed / wait ms      */
    int32_t b;             /* right                                 */
    int32_t c;             /* duration ms                           */
    double value;          /* param value                           */
    char name[24];         /* param name                            */
    char text[48];         /* say text                              */
} agent_plan_step_t;

/* Submit a parsed plan directly (used by the JSON parser and host tests). */
bool agent_runtime_submit_steps(const agent_plan_step_t *steps, uint8_t count,
                                char *err, size_t err_cap, uint32_t *plan_id);

/* Platform hook: true when the chassis is online, no e-stop and no blocking
 * fault.  Implemented by agent_runtime_hw.c on target; host tests stub it. */
bool agent_runtime_hw_safety_ok(void);

/* Test hook: replaces the voice/say sink (default: update last_action). */
void agent_runtime_set_say_handler(void (*fn)(const char *text));