/**
 * @file    robot_protocol.h
 * @brief   Protocol shared by the ESP32-S31 host and C30D STM32 controller.
 *
 * Frame:
 *   AA 55 | LEN | CMD | DATA[LEN] | CRC8
 *
 * CRC8 uses polynomial 0x07, initial value 0, and covers LEN + CMD + DATA.
 * Multi-byte integer fields use big-endian order.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RP_SOF1                  0xAAu
#define RP_SOF2                  0x55u
#define RP_MAX_PAYLOAD           32u
#define RP_MAX_FRAME             (RP_MAX_PAYLOAD + 5u)

/* ESP32-S31 -> C30D STM32 */
#define RP_CMD_DRIVE             0x10u
#define RP_CMD_ACTUATOR          0x11u
#define RP_CMD_ESTOP             0x12u
#define RP_CMD_HEARTBEAT         0x13u
#define RP_CMD_CLEAR_ESTOP       0x14u
#define RP_CMD_LINE              0x15u
#define RP_CMD_STOP_ALL          0x17u
#define RP_CMD_CALIBRATE_PAN     0x18u

/* C30D STM32 -> ESP32-S31 */
#define RP_CMD_CHASSIS_STATE     0x20u
#define RP_CMD_SENSOR_STATE      0x21u
#define RP_CMD_SAFETY_STATE      0x22u
#define RP_CMD_LINE_STATE        0x23u
#define RP_CMD_IMU_STATE         0x24u
#define RP_CMD_MOTOR_STATE       0x25u
#define RP_CMD_COMMAND_ACK       0x26u
#define RP_CMD_SET_PARAM         0x30u
#define RP_CMD_GET_PARAM         0x31u
#define RP_CMD_PARAM_STATE       0x33u
#define RP_PARAM_FRAME_LEN       5u

enum {
    RP_DRIVE_LEN = 2,
    RP_ACTUATOR_LEN = 3,
    RP_LINE_LEN = 2,
    RP_CHASSIS_STATE_LEN = 6,
    RP_SENSOR_STATE_LEN = 6,
    RP_SAFETY_STATE_LEN = 8,
    RP_LINE_STATE_LEN = 10,
    RP_IMU_STATE_LEN = 12,
    RP_MOTOR_STATE_LEN = 18,
    RP_COMMAND_ACK_LEN = 2
};

enum {
    RP_PAN_HOME_DEG = 270,
    RP_PAN_MAX_DEG = 540,
    RP_PAN_KNOWN = 1u << 0,
    RP_PAN_MOVING = 1u << 1,
    RP_PAN_BOOT_RETURN = 1u << 2
};

enum {
    RP_LINE_FLAG_ACTIVE      = 1u << 0,
    RP_LINE_FLAG_CCD_FOUND   = 1u << 1,
    RP_LINE_FLAG_CCD_VALID   = 1u << 2,
    RP_LINE_FLAG_OBSTACLE    = 1u << 3,
    RP_LINE_FLAG_REWIND      = 1u << 4,
    RP_LINE_FLAG_WAIT_CLEAR  = 1u << 5
};

enum {
    RP_PUSH_STOP = 0,
    RP_PUSH_EXTEND = 1,
    RP_PUSH_RETRACT = 2
};

enum {
    RP_ROBOT_IDLE = 0,
    RP_ROBOT_ACTIVE = 1,
    RP_ROBOT_ESTOP = 2,
    RP_ROBOT_FAULT = 3
};

/* Safety-state flags; several may be set at once. */
enum {
    RP_FAULT_ESTOP          = 1u << 0,
    RP_FAULT_CMD_TIMEOUT    = 1u << 1,
    RP_FAULT_GIMBAL_UNHOMED = 1u << 2,
    RP_FAULT_MOTION_STOP    = 1u << 3,
    RP_FAULT_IMU_OFFLINE    = 1u << 4,
    RP_FAULT_ULTRA_TIMEOUT  = 1u << 5,
    RP_FAULT_LOW_BATTERY    = 1u << 6,
    RP_FAULT_UART_OVERFLOW  = 1u << 7
};

typedef enum {
    RP_PARSE_NONE = 0,
    RP_PARSE_FRAME,
    RP_PARSE_CRC_ERROR,
    RP_PARSE_LENGTH_ERROR
} rp_parse_result_t;

typedef struct {
    int8_t left_percent;
    int8_t right_percent;
} rp_drive_cmd_t;

typedef struct {
    uint16_t pan_deg;
    uint8_t push_action;
} rp_actuator_cmd_t;

typedef struct {
    uint8_t enable;
    uint8_t speed_percent;
} rp_line_cmd_t;

typedef struct {
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    int16_t yaw_cdeg;
} rp_chassis_state_t;

typedef struct {
    uint16_t distance_cm;
    uint8_t push_top_limit;
    uint8_t push_bottom_limit;
    uint16_t battery_mv;
} rp_sensor_state_t;

typedef struct {
    uint8_t robot_state;
    uint8_t fault_flags;
    uint16_t uptime_s;
    uint8_t fw_major;
    uint8_t fw_minor;
    uint16_t last_cmd_age_ms;
} rp_safety_state_t;

typedef struct {
    uint8_t flags;
    uint8_t source;
    uint8_t ccd_center;
    uint8_t ccd_confidence;
    int8_t line_error;
    uint8_t lf_bits;
    int8_t left_command;
    int8_t right_command;
    uint8_t rewind_progress;
    uint8_t distance_cm;
} rp_line_state_t;

bool rp_line_has_signal(const rp_line_state_t *state);

typedef struct {
    int16_t gyro_x_ddps;
    int16_t gyro_y_ddps;
    int16_t gyro_z_ddps;
    int16_t accel_x_mg;
    int16_t accel_y_mg;
    int16_t accel_z_mg;
} rp_imu_state_t;

typedef struct {
    int16_t encoder_delta_a;
    int16_t encoder_delta_b;
    int16_t pwm_a;
    int16_t pwm_b;
    int16_t target_a_mm_s;
    int16_t target_b_mm_s;
    uint16_t pan_deg;
    uint8_t control_source;
    int8_t left_command;
    int8_t right_command;
    uint8_t pan_flags;
} rp_motor_state_t;

typedef struct {
    uint8_t command;
    uint8_t status;
} rp_command_ack_t;

typedef struct {
    uint8_t state;
    uint8_t len;
    uint8_t cmd;
    uint8_t idx;
    uint8_t crc;
    uint8_t payload[RP_MAX_PAYLOAD];
} rp_parser_t;

uint8_t rp_crc8(const uint8_t *data, size_t len);

size_t rp_encode_frame(uint8_t cmd,
                       const uint8_t *payload,
                       uint8_t len,
                       uint8_t *out,
                       size_t out_cap);

void rp_parser_init(rp_parser_t *parser);

rp_parse_result_t rp_parser_feed(rp_parser_t *parser,
                                 uint8_t byte,
                                 uint8_t *cmd_out,
                                 uint8_t *payload_out,
                                 uint8_t *len_out);

uint16_t rp_get_u16_be(const uint8_t *p);
int16_t rp_get_i16_be(const uint8_t *p);
void rp_put_u16_be(uint8_t *p, uint16_t value);
void rp_put_i16_be(uint8_t *p, int16_t value);

/*
 * Unpack contract (v3.9.8):
 *   1. *value is written only when the function returns true;
 *   2. on false (null pointer, wrong length, out-of-range field) *value must
 *      stay untouched, so the caller can keep using its previous value;
 *   3. call sites must check the return value before using the unpacked data.
 */
bool rp_pack_drive(const rp_drive_cmd_t *value, uint8_t out[RP_DRIVE_LEN]);
bool rp_unpack_drive(const uint8_t *data, uint8_t len, rp_drive_cmd_t *value);

bool rp_pack_actuator(const rp_actuator_cmd_t *value, uint8_t out[RP_ACTUATOR_LEN]);
bool rp_unpack_actuator(const uint8_t *data, uint8_t len, rp_actuator_cmd_t *value);

bool rp_pack_line_mode(const rp_line_cmd_t *value, uint8_t out[RP_LINE_LEN]);
bool rp_unpack_line_mode(const uint8_t *data, uint8_t len,
                         rp_line_cmd_t *value);

bool rp_pack_chassis_state(const rp_chassis_state_t *value,
                           uint8_t out[RP_CHASSIS_STATE_LEN]);
bool rp_unpack_chassis_state(const uint8_t *data,
                             uint8_t len,
                             rp_chassis_state_t *value);

bool rp_pack_sensor_state(const rp_sensor_state_t *value,
                          uint8_t out[RP_SENSOR_STATE_LEN]);
bool rp_unpack_sensor_state(const uint8_t *data,
                            uint8_t len,
                            rp_sensor_state_t *value);

bool rp_pack_safety_state(const rp_safety_state_t *value,
                          uint8_t out[RP_SAFETY_STATE_LEN]);
bool rp_unpack_safety_state(const uint8_t *data,
                            uint8_t len,
                            rp_safety_state_t *value);

bool rp_pack_line_state(const rp_line_state_t *value,
                        uint8_t out[RP_LINE_STATE_LEN]);
bool rp_unpack_line_state(const uint8_t *data,
                          uint8_t len,
                          rp_line_state_t *value);

bool rp_pack_imu_state(const rp_imu_state_t *value,
                       uint8_t out[RP_IMU_STATE_LEN]);
bool rp_unpack_imu_state(const uint8_t *data,
                         uint8_t len,
                         rp_imu_state_t *value);

bool rp_pack_motor_state(const rp_motor_state_t *value,
                         uint8_t out[RP_MOTOR_STATE_LEN]);
bool rp_unpack_motor_state(const uint8_t *data,
                           uint8_t len,
                           rp_motor_state_t *value);

/* Runtime control parameters shared with C30D (v3.10.0). */
typedef enum {
    RP_PARAM_LINE_BASE_PERCENT = 1,
    RP_PARAM_LINE_TURN_GAIN = 2,
    RP_PARAM_SPEED_KP = 3,
    RP_PARAM_SPEED_KI = 4,
    RP_PARAM_MAX_SPEED_MMS = 5,
    RP_PARAM_COUNT = 5,
} rp_param_id_t;

typedef struct {
    uint8_t id;
    int32_t value;
} rp_param_t;

bool rp_param_range(uint8_t id, int32_t *min_value, int32_t *max_value,
                    int32_t *default_value);
bool rp_param_valid(uint8_t id, int32_t value);
bool rp_pack_param(uint8_t id, int32_t value,
                   uint8_t out[RP_PARAM_FRAME_LEN]);
bool rp_unpack_param(const uint8_t *data, uint8_t len, uint8_t *id,
                     int32_t *value);

bool rp_pack_command_ack(const rp_command_ack_t *value,
                         uint8_t out[RP_COMMAND_ACK_LEN]);
bool rp_unpack_command_ack(const uint8_t *data,
                           uint8_t len,
                           rp_command_ack_t *value);

#ifdef __cplusplus
}
#endif
