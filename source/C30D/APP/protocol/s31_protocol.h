#pragma once

#include <stdint.h>

#define S31_SOF1 0xAAu
#define S31_SOF2 0x55u
#define S31_MAX_PAYLOAD 32u
#define S31_MAX_FRAME (S31_MAX_PAYLOAD + 5u)

#define S31_CMD_DRIVE                0x10u
#define S31_CMD_ACTUATOR             0x11u
#define S31_CMD_ESTOP                0x12u
#define S31_CMD_HEARTBEAT            0x13u
#define S31_CMD_CLEAR_ESTOP          0x14u
#define S31_CMD_LINE                 0x15u
#define S31_CMD_STOP_ALL             0x17u
#define S31_CMD_CALIBRATE_PAN        0x18u
#define S31_CMD_COMMAND_ACK          0x26u

/* v3.10.0 runtime parameter service (payload: id u8 + value i32 big-endian) */
#define S31_CMD_SET_PARAM            0x30u
#define S31_CMD_GET_PARAM            0x31u
#define S31_CMD_PARAM_STATE          0x33u
#define C30D_PARAM_FRAME_LEN         5u

#define S31_CMD_CHASSIS_STATE        0x20u
#define S31_CMD_SENSOR_STATE         0x21u
#define S31_CMD_SAFETY_STATE         0x22u
#define S31_CMD_LINE_STATE           0x23u
#define S31_CMD_IMU_STATE            0x24u
#define S31_CMD_MOTOR_STATE          0x25u

typedef enum
{
    S31_FRAME_NONE = 0,
    S31_FRAME_READY,
    S31_FRAME_BAD_CRC
} s31_frame_result_t;

typedef enum
{
    S31_RX_HEADER_FIRST = 0,
    S31_RX_HEADER_SECOND,
    S31_RX_LENGTH,
    S31_RX_COMMAND,
    S31_RX_DATA
} s31_rx_state_t;

typedef struct
{
    s31_rx_state_t state;
    uint8_t length;
    uint8_t command;
    uint8_t crc;
    uint8_t index;
    uint8_t payload[S31_MAX_PAYLOAD];
} s31_frame_parser_t;

/* Returns the encoded byte count, or 0 for an invalid payload. */
uint8_t s31_frame_encode(uint8_t command, const uint8_t *payload, uint8_t length,
                         uint8_t frame[S31_MAX_FRAME]);

/* Initialize with {0}. On READY the fields remain valid until the next feed. */
s31_frame_result_t s31_frame_feed(s31_frame_parser_t *parser, uint8_t byte);
