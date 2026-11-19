#include "control.h"

#include "agent/agent_runtime.h"
#include "app/app_config.h"
#include "app/app_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "bus/robot_bus.h"
#include "transport/robot_link.h"
#include "protocol/robot_protocol.h"

#define FIELD_LINE_SPEED          15u
#define FIELD_LINE_DURATION_US   1200000
#define FIELD_DRIVE_SPEED         12
#define FIELD_DRIVE_DURATION_US   350000
#define FIELD_MIN_DISTANCE_CM     25u
#define INSPECT_TIMEOUT_US        3500000
#define INSPECT_DWELL_US          250000
#define CLEAR_ESTOP_GRACE_US      500000

typedef struct {
    td_mode_t mode;
    uint32_t generation;
    uint32_t stop_ack_generation;
    unsigned stop_requests;
    int8_t left;
    int8_t right;
    uint8_t line_speed;
    int64_t manual_deadline_us;
    int64_t field_deadline_us;
    bool field_takeover;
    int64_t ignore_remote_estop_until_us;
    td_mode_t inspect_return;
    uint8_t inspect_phase;
    bool inspect_target_sent;
    int64_t inspect_sent_us;
    int64_t inspect_deadline_us;
    int64_t inspect_arrived_us;
    uint32_t last_drive_ms;
} motion_state_t;

typedef enum { SEND_STOP, SEND_DRIVE, SEND_LINE, SEND_PAN, SEND_ESTOP } command_kind_t;
typedef struct {
    command_kind_t kind;
    int8_t left;
    int8_t right;
    uint8_t speed;
    uint16_t pan;
    uint8_t push;
} motion_command_t;

static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_motion_tx;
static motion_state_t s_motion;
static int32_t s_params[RP_PARAM_COUNT + 1];

static const uint16_t s_inspect_angles[] = {
    RP_PAN_HOME_DEG, RP_PAN_HOME_DEG - 45u,
    RP_PAN_HOME_DEG + 45u, RP_PAN_HOME_DEG,
};

static void lock_motion(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
}

static void unlock_motion(void)
{
    xSemaphoreGive(s_mutex);
}

/* The state lock precedes the app-state lock. USB waits use only s_motion_tx. */
static void publish_motion_locked(int8_t left, int8_t right)
{
    app_state_lock();
    td_status_t *status = app_state_mutable();
    status->mode = s_motion.mode;
    status->out_left = left;
    status->out_right = right;
    status->inspect_phase = s_motion.mode == TD_MODE_INSPECT ? s_motion.inspect_phase : 0u;
    app_state_unlock();
}

static uint32_t enter_mode_locked(td_mode_t mode)
{
    s_motion.mode = mode;
    if (++s_motion.generation == 0u) {
        ++s_motion.generation;
    }
    s_motion.manual_deadline_us = 0;
    s_motion.field_deadline_us = 0;
    s_motion.stop_ack_generation = 0;
    return s_motion.generation;
}

static bool can_start_locked(void)
{
    return s_motion.mode != TD_MODE_ESTOP && s_motion.stop_requests == 0u;
}

static bool send_command(const motion_command_t *cmd)
{
    switch (cmd->kind) {
    case SEND_STOP: return robot_link_send_stop_all();
    case SEND_DRIVE: return robot_link_send_drive(cmd->left, cmd->right);
    case SEND_LINE: return robot_link_send_line_mode(true, cmd->speed);
    case SEND_PAN: return robot_link_send_actuator(cmd->pan, cmd->push);
    case SEND_ESTOP: return robot_link_emergency_stop();
    }
    return false;
}

/* An older motion frame may finish before STOP, but can never be sent after it.
 * An ACK from a superseded request cannot publish or restore an old mode. */
static const char *field_guard(const td_status_t *status, int64_t now_us, bool line);

static bool send_ordered(uint32_t generation, const motion_command_t *command)
{
    xSemaphoreTake(s_motion_tx, portMAX_DELAY);
    td_status_t status;
    app_state_snapshot(&status);
    int64_t now_us = app_now_us();
    bool wheel_command = command->kind == SEND_DRIVE || command->kind == SEND_LINE;
    bool lease_ok = agent_runtime_takeover_ok();
    lock_motion();
    bool current = s_motion.generation == generation;
    if (current && wheel_command) {
        current = status.stm32_online &&
                  (command->kind != SEND_DRIVE || now_us < s_motion.manual_deadline_us);
        if (current && s_motion.field_deadline_us != 0) {
            current = now_us < s_motion.field_deadline_us &&
                      (!s_motion.field_takeover || lease_ok) &&
                      field_guard(&status, now_us, command->kind == SEND_LINE) == NULL;
        }
    }
    unlock_motion();
    bool ok = current && send_command(command);
    lock_motion();
    current = s_motion.generation == generation;
    if (ok && current &&
        (command->kind == SEND_ESTOP || command->kind == SEND_STOP)) {
        s_motion.stop_ack_generation = generation;
    }
    unlock_motion();
    xSemaphoreGive(s_motion_tx);
    return ok && current;
}

static void abort_if_current(uint32_t generation)
{
    lock_motion();
    bool field = s_motion.generation == generation && s_motion.field_deadline_us != 0;
    if (s_motion.generation == generation) {
        enter_mode_locked(TD_MODE_IDLE);
        publish_motion_locked(0, 0);
    }
    unlock_motion();
    if (field) {
        rb_log_event("field", "motion_end", "C30D command not acknowledged");
    }
}

static bool telemetry_fresh(int64_t now_us, int64_t rx_us)
{
    return rx_us > 0 && rx_us <= now_us &&
           now_us - rx_us <= (int64_t)TD_STM32_ONLINE_MS * 1000;
}

static bool remote_estop(const td_status_t *status)
{
    return status->safety.robot_state == RP_ROBOT_ESTOP ||
           (status->safety.fault_flags & RP_FAULT_ESTOP) != 0u;
}

static bool pan_available(const td_status_t *status, int64_t now_us)
{
    return status->stm32_online && telemetry_fresh(now_us, status->stm32_motor_rx_us) &&
           (status->motor.pan_flags & (RP_PAN_KNOWN | RP_PAN_BOOT_RETURN)) == RP_PAN_KNOWN;
}

static bool motors_stationary(const rp_motor_state_t *motor)
{
    return motor->control_source == 0u &&
           motor->pwm_a == 0 && motor->pwm_b == 0 &&
           motor->encoder_delta_a == 0 && motor->encoder_delta_b == 0 &&
           (motor->pan_flags & (RP_PAN_MOVING | RP_PAN_BOOT_RETURN)) == 0u;
}

static bool field_stationary(const td_status_t *status, int64_t now_us)
{
    return status->stm32_online && telemetry_fresh(now_us, status->stm32_motor_rx_us) &&
           (status->mode == TD_MODE_IDLE || status->mode == TD_MODE_ESTOP) &&
           motors_stationary(&status->motor);
}

static const char *field_guard(const td_status_t *s, int64_t now_us, bool line)
{
    if (!s->sta_connected) {
        return "Wi-Fi disconnected";
    }
    if (!s->stm32_online || !telemetry_fresh(now_us, s->stm32_sensor_rx_us) ||
        !telemetry_fresh(now_us, s->stm32_safety_rx_us) ||
        !telemetry_fresh(now_us, s->stm32_motor_rx_us) ||
        (line && !telemetry_fresh(now_us, s->stm32_line_rx_us))) {
        return "C30D telemetry stale";
    }
    const uint8_t faults = RP_FAULT_ESTOP | RP_FAULT_LOW_BATTERY |
                           RP_FAULT_IMU_OFFLINE | RP_FAULT_UART_OVERFLOW;
    if (remote_estop(s) || s->safety.robot_state == RP_ROBOT_FAULT ||
        (s->safety.fault_flags & faults) != 0u ||
        s->sensor.battery_mv < 10000u || s->sensor.battery_mv == UINT16_MAX) {
        return "C30D safety fault";
    }
    if (s->sensor.distance_cm <= FIELD_MIN_DISTANCE_CM ||
        s->sensor.distance_cm == UINT16_MAX ||
        (s->safety.fault_flags & RP_FAULT_ULTRA_TIMEOUT) != 0u) {
        return "obstacle within 25 cm or no valid range";
    }
    if ((s->motor.pan_flags & (RP_PAN_KNOWN | RP_PAN_MOVING | RP_PAN_BOOT_RETURN)) != RP_PAN_KNOWN) {
        return "camera pan is moving or its position is unknown";
    }
    const uint8_t recovery = RP_LINE_FLAG_OBSTACLE | RP_LINE_FLAG_REWIND | RP_LINE_FLAG_WAIT_CLEAR;
    if (line && (!rp_line_has_signal(&s->line_state) || (s->line_state.flags & recovery) != 0u)) {
        return "line unavailable or obstacle recovery active";
    }
    return NULL;
}

static bool begin_driving(const motion_command_t *command, uint32_t hold_ms)
{
    td_status_t status;
    app_state_snapshot(&status);
    if (s_mutex == NULL || !status.stm32_online) {
        return false;
    }
    lock_motion();
    if (!can_start_locked()) {
        unlock_motion();
        return false;
    }
    bool line = command->kind == SEND_LINE;
    uint32_t generation = enter_mode_locked(line ? TD_MODE_LINE : TD_MODE_MANUAL);
    if (line) {
        s_motion.line_speed = command->speed;
    } else {
        s_motion.left = command->left;
        s_motion.right = command->right;
        s_motion.manual_deadline_us = app_now_us() + (int64_t)hold_ms * 1000;
    }
    s_motion.last_drive_ms = app_now_ms();
    publish_motion_locked(command->left, command->right);
    unlock_motion();

    if (send_ordered(generation, command)) {
        return true;
    }
    abort_if_current(generation);
    return false;
}

void control_start_line(void)
{
    (void)control_start_line_speed(TD_LINE_MODULE_SPEED);
}

bool control_start_line_speed(uint8_t speed)
{
    motion_command_t command = {
        .kind = SEND_LINE,
        .speed = speed < 5u ? 5u : (speed > TD_USER_TRACK_MAX_PERCENT ? TD_USER_TRACK_MAX_PERCENT : speed),
    };
    return begin_driving(&command, 0);
}

bool control_manual(int8_t left, int8_t right, uint32_t hold_ms)
{
    motion_command_t command = {
        .kind = SEND_DRIVE,
        .left = left > 100 ? 100 : (left < -100 ? -100 : left),
        .right = right > 100 ? 100 : (right < -100 ? -100 : right),
    };
    if (hold_ms < 100u) {
        hold_ms = 100u;
    } else if (hold_ms > TD_MANUAL_MAX_HOLD_MS) {
        hold_ms = TD_MANUAL_MAX_HOLD_MS;
    }
    return begin_driving(&command, hold_ms);
}

bool control_stop(void)
{
    if (s_mutex == NULL) {
        return false;
    }
    lock_motion();
    ++s_motion.stop_requests;
    bool estopped = s_motion.mode == TD_MODE_ESTOP;
    uint32_t generation = enter_mode_locked(estopped ? TD_MODE_ESTOP : TD_MODE_IDLE);
    publish_motion_locked(0, 0);
    unlock_motion();

    motion_command_t command = {.kind = estopped ? SEND_ESTOP : SEND_STOP};
    bool ok = send_ordered(generation, &command);
    if (!ok && !estopped) {
        lock_motion();
        bool current = s_motion.generation == generation;
        unlock_motion();
        if (current && robot_link_emergency_stop()) {
            lock_motion();
            if (s_motion.generation == generation) {
                enter_mode_locked(TD_MODE_ESTOP);
                publish_motion_locked(0, 0);
            }
            unlock_motion();
            ok = true;
        }
    }

    lock_motion();
    --s_motion.stop_requests;
    unlock_motion();
    return ok;
}

bool control_halt(void)
{
    if (s_mutex == NULL) {
        return false;
    }
    /* Block graph actions while stopping and discarding their pending work. */
    lock_motion();
    ++s_motion.stop_requests;
    unlock_motion();
    bool ok = control_stop();
    agent_runtime_abort_motion("stop requested");
    lock_motion();
    --s_motion.stop_requests;
    unlock_motion();
    return ok;
}

bool control_estop(void)
{
    if (s_mutex == NULL) {
        return robot_link_emergency_stop();
    }
    lock_motion();
    s_motion.ignore_remote_estop_until_us = 0;
    enter_mode_locked(TD_MODE_ESTOP);
    publish_motion_locked(0, 0);
    unlock_motion();

    bool ok = robot_link_emergency_stop();
    agent_runtime_abort_motion("emergency stop");
    return ok;
}

bool control_clear_estop(void)
{
    if (s_mutex == NULL) {
        return false;
    }
    xSemaphoreTake(s_motion_tx, portMAX_DELAY);
    lock_motion();
    uint32_t generation = s_motion.generation;
    bool available = s_motion.stop_requests == 0u;
    unlock_motion();

    bool ok = available && robot_link_clear_estop();
    lock_motion();
    bool superseded = generation != s_motion.generation;
    if (ok && !superseded) {
        enter_mode_locked(TD_MODE_IDLE);
        s_motion.ignore_remote_estop_until_us = app_now_us() + CLEAR_ESTOP_GRACE_US;
        publish_motion_locked(0, 0);
    }
    bool reassert = ok && superseded && s_motion.mode == TD_MODE_ESTOP;
    unlock_motion();
    if (reassert) {
        (void)robot_link_emergency_stop();
    }
    xSemaphoreGive(s_motion_tx);
    return ok && !superseded;
}

bool control_field_stationary(void)
{
    td_status_t status;
    app_state_snapshot(&status);
    return field_stationary(&status, app_now_us());
}

const char *control_field_move(field_action_t action)
{
    if (s_mutex == NULL) {
        return "control unavailable";
    }
    if (action == FIELD_BACKWARD_SHORT) {
        return "reverse blocked: no rear range sensor";
    }
    bool line = action == FIELD_FOLLOW_LINE;
    if (!line && action != FIELD_FORWARD_SHORT &&
        action != FIELD_TURN_LEFT_SMALL && action != FIELD_TURN_RIGHT_SMALL) {
        return "unsupported movement";
    }

    agent_state_t agent;
    agent_runtime_state_snapshot(&agent);
    if (agent.plan_running) {
        return "operator plan is running";
    }
    td_status_t status;
    app_state_snapshot(&status);
    int64_t now_us = app_now_us();
    const char *reason = field_guard(&status, now_us, line);
    if (reason != NULL) {
        return reason;
    }
    lock_motion();
    if (!can_start_locked() || s_motion.mode != TD_MODE_IDLE || !field_stationary(&status, now_us)) {
        unlock_motion();
        return "wait for the chassis to stop";
    }
    enter_mode_locked(line ? TD_MODE_LINE : TD_MODE_MANUAL);
    s_motion.field_takeover = agent.mode == AGENT_MODE_TAKEOVER;
    s_motion.field_deadline_us = now_us + (line ? FIELD_LINE_DURATION_US : FIELD_DRIVE_DURATION_US);
    s_motion.manual_deadline_us = s_motion.field_deadline_us;
    s_motion.line_speed = FIELD_LINE_SPEED;
    s_motion.left = action == FIELD_TURN_LEFT_SMALL ? -FIELD_DRIVE_SPEED : FIELD_DRIVE_SPEED;
    s_motion.right = action == FIELD_TURN_RIGHT_SMALL ? -FIELD_DRIVE_SPEED : FIELD_DRIVE_SPEED;
    s_motion.last_drive_ms = app_now_ms() - TD_DRIVE_PERIOD_MS;
    publish_motion_locked(0, 0);
    unlock_motion();
    return NULL;
}

void control_field_cancel(void)
{
    if (s_mutex == NULL) {
        return;
    }
    lock_motion();
    if (s_motion.field_deadline_us != 0) {
        enter_mode_locked(TD_MODE_IDLE);
        publish_motion_locked(0, 0);
    }
    unlock_motion();
}

static void publish_pan_target(uint16_t angle, uint8_t push)
{
    app_state_lock();
    td_status_t *status = app_state_mutable();
    status->pan_deg = angle;
    status->push_action = push;
    app_state_unlock();
}

bool control_actuator(uint16_t angle, uint8_t push)
{
    if (s_mutex == NULL || angle > RP_PAN_MAX_DEG || push > RP_PUSH_RETRACT) {
        return false;
    }
    td_status_t status;
    app_state_snapshot(&status);
    if (!pan_available(&status, app_now_us())) {
        return false;
    }

    lock_motion();
    bool available = can_start_locked() && s_motion.mode != TD_MODE_INSPECT;
    uint32_t generation = s_motion.generation;
    unlock_motion();
    motion_command_t command = {.kind = SEND_PAN, .pan = angle, .push = push};
    if (!available || !send_ordered(generation, &command)) {
        return false;
    }
    publish_pan_target(angle, push);
    return true;
}

bool control_calibrate_pan(void)
{
    if (s_mutex == NULL) {
        return false;
    }
    td_status_t status;
    app_state_snapshot(&status);
    if (!field_stationary(&status, app_now_us())) {
        return false;
    }
    xSemaphoreTake(s_motion_tx, portMAX_DELAY);
    lock_motion();
    bool available = s_motion.mode == TD_MODE_IDLE && can_start_locked();
    unlock_motion();
    bool ok = available && robot_link_send_calibrate_pan();
    if (ok) {
        publish_pan_target(RP_PAN_HOME_DEG, RP_PUSH_STOP);
    }
    xSemaphoreGive(s_motion_tx);
    return ok;
}

bool control_start_inspect(void)
{
    if (s_mutex == NULL) {
        return false;
    }
    td_status_t status;
    app_state_snapshot(&status);
    if (!pan_available(&status, app_now_us())) {
        return false;
    }
    lock_motion();
    if (!can_start_locked()) {
        unlock_motion();
        return false;
    }
    s_motion.inspect_return = s_motion.mode == TD_MODE_LINE &&
                              s_motion.field_deadline_us == 0 ? TD_MODE_LINE : TD_MODE_IDLE;
    uint32_t generation = enter_mode_locked(TD_MODE_INSPECT);
    s_motion.inspect_phase = 0u;
    s_motion.inspect_target_sent = false;
    s_motion.inspect_arrived_us = 0;
    s_motion.inspect_deadline_us = app_now_us() + INSPECT_TIMEOUT_US;
    publish_motion_locked(0, 0);
    unlock_motion();

    const motion_command_t stop = {.kind = SEND_STOP};
    if (send_ordered(generation, &stop)) {
        return true;
    }
    abort_if_current(generation);
    return false;
}

static bool tick_inspect_locked(const td_status_t *status, int64_t now_us,
                                motion_command_t *command)
{
    uint16_t angle = s_inspect_angles[s_motion.inspect_phase];
    if (!pan_available(status, now_us) || now_us >= s_motion.inspect_deadline_us) {
        enter_mode_locked(TD_MODE_IDLE);
        return false;
    }
    if (s_motion.stop_ack_generation != s_motion.generation) {
        return false;
    }
    if (!s_motion.inspect_target_sent) {
        if (!motors_stationary(&status->motor)) {
            return false;
        }
        s_motion.inspect_target_sent = true;
        s_motion.inspect_sent_us = now_us;
        *command = (motion_command_t){.kind = SEND_PAN, .pan = angle, .push = RP_PUSH_STOP};
        return true;
    }
    if (status->stm32_motor_rx_us <= s_motion.inspect_sent_us ||
        !motors_stationary(&status->motor) || status->motor.pan_deg != angle) {
        s_motion.inspect_arrived_us = 0;
        return false;
    }
    if (s_motion.inspect_arrived_us == 0) {
        s_motion.inspect_arrived_us = now_us;
    } else if (now_us - s_motion.inspect_arrived_us >= INSPECT_DWELL_US) {
        if (++s_motion.inspect_phase == sizeof(s_inspect_angles) / sizeof(s_inspect_angles[0])) {
            enter_mode_locked(s_motion.inspect_return);
        } else {
            s_motion.inspect_target_sent = false;
            s_motion.inspect_arrived_us = 0;
            s_motion.inspect_deadline_us = now_us + INSPECT_TIMEOUT_US;
        }
    }
    return false;
}

static void control_tick(void)
{
    td_status_t status;
    app_state_snapshot(&status);
    const int64_t now_us = app_now_us();
    const uint32_t now_ms = app_now_ms();
    bool lease_ok = agent_runtime_takeover_ok();
    bool send = false;
    bool cancel_automation = false;
    motion_command_t command = {0};
    const char *field_end = NULL;

    lock_motion();
    if (status.stm32_online && telemetry_fresh(now_us, status.stm32_safety_rx_us) &&
        remote_estop(&status) && now_us >= s_motion.ignore_remote_estop_until_us &&
        s_motion.mode != TD_MODE_ESTOP) {
        enter_mode_locked(TD_MODE_ESTOP);
        cancel_automation = true;
    }
    if (s_motion.field_deadline_us != 0) {
        field_end = field_guard(&status, now_us, s_motion.mode == TD_MODE_LINE);
        if (s_motion.field_takeover && !lease_ok) {
            field_end = "lease ended";
        } else if (field_end == NULL && now_us >= s_motion.field_deadline_us) {
            field_end = "movement window completed";
        }
    }
    if (field_end != NULL ||
        (s_motion.mode == TD_MODE_MANUAL && now_us >= s_motion.manual_deadline_us) ||
        (!status.stm32_online && s_motion.mode != TD_MODE_ESTOP && s_motion.mode != TD_MODE_IDLE)) {
        enter_mode_locked(TD_MODE_IDLE);
    }

    int8_t left = 0;
    int8_t right = 0;
    if (s_motion.mode == TD_MODE_INSPECT) {
        send = tick_inspect_locked(&status, now_us, &command);
    }
    if (s_motion.mode == TD_MODE_MANUAL) {
        left = s_motion.left;
        right = s_motion.right;
        command = (motion_command_t){.kind = SEND_DRIVE, .left = left, .right = right};
    } else if (s_motion.mode == TD_MODE_LINE) {
        left = status.line_state.left_command;
        right = status.line_state.right_command;
        command = (motion_command_t){.kind = SEND_LINE, .speed = s_motion.line_speed};
    } else if (s_motion.mode != TD_MODE_INSPECT) {
        command.kind = s_motion.mode == TD_MODE_ESTOP ? SEND_ESTOP : SEND_STOP;
    }
    if (!status.stm32_online) {
        s_motion.stop_ack_generation = 0;
    } else if (s_motion.mode != TD_MODE_INSPECT && s_motion.stop_requests == 0u) {
        bool stopped = s_motion.mode == TD_MODE_IDLE || s_motion.mode == TD_MODE_ESTOP;
        send = stopped ? s_motion.stop_ack_generation != s_motion.generation :
               now_ms - s_motion.last_drive_ms >= TD_DRIVE_PERIOD_MS;
    }
    if (send) {
        s_motion.last_drive_ms = now_ms;
    }
    uint32_t generation = s_motion.generation;
    publish_motion_locked(left, right);
    unlock_motion();

    if (send) {
        bool ok = send_ordered(generation, &command);
        if (!ok && (command.kind == SEND_DRIVE || command.kind == SEND_LINE || command.kind == SEND_PAN)) {
            abort_if_current(generation);
        } else if (ok && command.kind == SEND_PAN) {
            publish_pan_target(command.pan, command.push);
        }
    }
    if (field_end != NULL) {
        rb_log_event("field", "motion_end", field_end);
    }
    if (cancel_automation) {
        agent_runtime_abort_motion("C30D emergency stop");
    }
}

#ifdef CONTROL_HOST_TEST
void control_sim_tick(void)
{
    control_tick();
}
#endif

static void control_task(void *arg)
{
    (void)arg;
    while (true) {
        control_tick();
        vTaskDelay(pdMS_TO_TICKS(TD_CONTROL_PERIOD_MS));
    }
}

esp_err_t control_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_motion_tx = xSemaphoreCreateMutex();
    if (s_mutex == NULL || s_motion_tx == NULL) {
        if (s_mutex != NULL) {
            vSemaphoreDelete(s_mutex);
        }
        if (s_motion_tx != NULL) {
            vSemaphoreDelete(s_motion_tx);
        }
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_motion = (motion_state_t){.generation = 1u, .line_speed = TD_LINE_MODULE_SPEED};
    for (uint8_t id = 1u; id <= RP_PARAM_COUNT; ++id) {
        (void)rp_param_range(id, NULL, NULL, &s_params[id]);
    }
    if (xTaskCreate(control_task, "control", 6144, NULL, 7, NULL) != pdPASS) {
        vSemaphoreDelete(s_motion_tx);
        vSemaphoreDelete(s_mutex);
        s_motion_tx = NULL;
        s_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool control_set_param(uint8_t id, int32_t value)
{
    if (s_mutex == NULL || !rp_param_valid(id, value) || !robot_link_send_set_param(id, value)) {
        return false;
    }
    control_param_state(id, value);
    return true;
}

bool control_get_param(uint8_t id, int32_t *value)
{
    if (s_mutex == NULL || value == NULL || id < 1u || id > RP_PARAM_COUNT) {
        return false;
    }
    lock_motion();
    *value = s_params[id];
    unlock_motion();
    return true;
}

void control_param_state(uint8_t id, int32_t value)
{
    if (s_mutex != NULL && rp_param_valid(id, value)) {
        lock_motion();
        s_params[id] = value;
        unlock_motion();
    }
}
