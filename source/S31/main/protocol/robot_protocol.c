#include "robot_protocol.h"

#include <string.h>

static uint8_t crc_update(uint8_t crc, uint8_t value)
{
    uint8_t bit;

    crc ^= value;
    for (bit = 0; bit < 8u; ++bit) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                            : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t rp_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    size_t i;

    if (data == NULL) {
        return 0;
    }

    for (i = 0; i < len; ++i) {
        crc = crc_update(crc, data[i]);
    }
    return crc;
}

size_t rp_encode_frame(uint8_t cmd,
                       const uint8_t *payload,
                       uint8_t len,
                       uint8_t *out,
                       size_t out_cap)
{
    size_t frame_len;

    if (out == NULL || len > RP_MAX_PAYLOAD || (len > 0u && payload == NULL)) {
        return 0;
    }

    frame_len = (size_t)len + 5u;
    if (out_cap < frame_len) {
        return 0;
    }

    out[0] = RP_SOF1;
    out[1] = RP_SOF2;
    out[2] = len;
    out[3] = cmd;
    if (len > 0u) {
        memcpy(&out[4], payload, len);
    }
    out[4u + len] = rp_crc8(&out[2], (size_t)len + 2u);
    return frame_len;
}

void rp_parser_init(rp_parser_t *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

rp_parse_result_t rp_parser_feed(rp_parser_t *parser,
                                 uint8_t byte,
                                 uint8_t *cmd_out,
                                 uint8_t *payload_out,
                                 uint8_t *len_out)
{
    rp_parse_result_t result = RP_PARSE_NONE;

    if (parser == NULL) {
        return result;
    }

    switch (parser->state) {
    case 0:
        if (byte == RP_SOF1) {
            parser->state = 1;
        }
        break;

    case 1:
        if (byte == RP_SOF2) {
            parser->state = 2;
        } else {
            parser->state = (byte == RP_SOF1) ? 1u : 0u;
        }
        break;

    case 2:
        parser->len = byte;
        parser->crc = crc_update(0, byte);
        parser->idx = 0;
        if (parser->len > RP_MAX_PAYLOAD) {
            rp_parser_init(parser);
            result = RP_PARSE_LENGTH_ERROR;
        } else {
            parser->state = 3;
        }
        break;

    case 3:
        parser->cmd = byte;
        parser->crc = crc_update(parser->crc, byte);
        parser->state = 4;
        break;

    case 4:
        if (parser->idx < parser->len) {
            parser->payload[parser->idx++] = byte;
            parser->crc = crc_update(parser->crc, byte);
        } else if (byte == parser->crc) {
            if (cmd_out != NULL) {
                *cmd_out = parser->cmd;
            }
            if (payload_out != NULL && parser->len > 0u) {
                memcpy(payload_out, parser->payload, parser->len);
            }
            if (len_out != NULL) {
                *len_out = parser->len;
            }
            result = RP_PARSE_FRAME;
            rp_parser_init(parser);
        } else {
            result = RP_PARSE_CRC_ERROR;
            rp_parser_init(parser);
        }
        break;

    default:
        rp_parser_init(parser);
        break;
    }

    return result;
}

uint16_t rp_get_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

int16_t rp_get_i16_be(const uint8_t *p)
{
    return (int16_t)rp_get_u16_be(p);
}

void rp_put_u16_be(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

void rp_put_i16_be(uint8_t *p, int16_t value)
{
    rp_put_u16_be(p, (uint16_t)value);
}

bool rp_pack_drive(const rp_drive_cmd_t *value, uint8_t out[RP_DRIVE_LEN])
{
    if (value == NULL || out == NULL) {
        return false;
    }
    out[0] = (uint8_t)value->left_percent;
    out[1] = (uint8_t)value->right_percent;
    return true;
}

bool rp_unpack_drive(const uint8_t *data, uint8_t len, rp_drive_cmd_t *value)
{
    int8_t left;
    int8_t right;

    if (data == NULL || value == NULL || len != RP_DRIVE_LEN) {
        return false;
    }
    left = (int8_t)data[0];
    right = (int8_t)data[1];
    if (left < -100 || left > 100 || right < -100 || right > 100) {
        return false;
    }
    value->left_percent = left;
    value->right_percent = right;
    return true;
}

bool rp_pack_actuator(const rp_actuator_cmd_t *value,
                      uint8_t out[RP_ACTUATOR_LEN])
{
    if (value == NULL || out == NULL || value->pan_deg > RP_PAN_MAX_DEG ||
        value->push_action > RP_PUSH_RETRACT) {
        return false;
    }
    rp_put_u16_be(out, value->pan_deg);
    out[2] = value->push_action;
    return true;
}

bool rp_pack_line_mode(const rp_line_cmd_t *value, uint8_t out[RP_LINE_LEN])
{
    if (value == NULL || out == NULL || value->enable > 1u ||
        value->speed_percent < 5u || value->speed_percent > 100u) {
        return false;
    }
    out[0] = value->enable;
    out[1] = value->speed_percent;
    return true;
}

bool rp_unpack_line_mode(const uint8_t *data, uint8_t len,
                         rp_line_cmd_t *value)
{
    if (data == NULL || value == NULL || len != RP_LINE_LEN ||
        data[0] > 1u || data[1] < 5u || data[1] > 100u) {
        return false;
    }
    value->enable = data[0];
    value->speed_percent = data[1];
    return true;
}

bool rp_unpack_actuator(const uint8_t *data,
                        uint8_t len,
                        rp_actuator_cmd_t *value)
{
    if (data == NULL || value == NULL || len != RP_ACTUATOR_LEN ||
        rp_get_u16_be(data) > RP_PAN_MAX_DEG || data[2] > RP_PUSH_RETRACT) {
        return false;
    }
    value->pan_deg = rp_get_u16_be(data);
    value->push_action = data[2];
    return true;
}

bool rp_pack_chassis_state(const rp_chassis_state_t *value,
                           uint8_t out[RP_CHASSIS_STATE_LEN])
{
    if (value == NULL || out == NULL) {
        return false;
    }
    rp_put_i16_be(&out[0], value->left_speed_mm_s);
    rp_put_i16_be(&out[2], value->right_speed_mm_s);
    rp_put_i16_be(&out[4], value->yaw_cdeg);
    return true;
}

bool rp_unpack_chassis_state(const uint8_t *data,
                             uint8_t len,
                             rp_chassis_state_t *value)
{
    if (data == NULL || value == NULL || len != RP_CHASSIS_STATE_LEN) {
        return false;
    }
    value->left_speed_mm_s = rp_get_i16_be(&data[0]);
    value->right_speed_mm_s = rp_get_i16_be(&data[2]);
    value->yaw_cdeg = rp_get_i16_be(&data[4]);
    return true;
}

bool rp_pack_sensor_state(const rp_sensor_state_t *value,
                          uint8_t out[RP_SENSOR_STATE_LEN])
{
    if (value == NULL || out == NULL) {
        return false;
    }
    rp_put_u16_be(&out[0], value->distance_cm);
    out[2] = value->push_top_limit;
    out[3] = value->push_bottom_limit;
    rp_put_u16_be(&out[4], value->battery_mv);
    return true;
}

bool rp_unpack_sensor_state(const uint8_t *data,
                            uint8_t len,
                            rp_sensor_state_t *value)
{
    if (data == NULL || value == NULL || len != RP_SENSOR_STATE_LEN) {
        return false;
    }
    value->distance_cm = rp_get_u16_be(&data[0]);
    value->push_top_limit = data[2];
    value->push_bottom_limit = data[3];
    value->battery_mv = rp_get_u16_be(&data[4]);
    return true;
}

bool rp_pack_safety_state(const rp_safety_state_t *value,
                          uint8_t out[RP_SAFETY_STATE_LEN])
{
    if (value == NULL || out == NULL) {
        return false;
    }
    out[0] = value->robot_state;
    out[1] = value->fault_flags;
    rp_put_u16_be(&out[2], value->uptime_s);
    out[4] = value->fw_major;
    out[5] = value->fw_minor;
    rp_put_u16_be(&out[6], value->last_cmd_age_ms);
    return true;
}

bool rp_unpack_safety_state(const uint8_t *data,
                            uint8_t len,
                            rp_safety_state_t *value)
{
    if (data == NULL || value == NULL || len != RP_SAFETY_STATE_LEN ||
        data[0] > RP_ROBOT_FAULT) {
        return false;
    }
    value->robot_state = data[0];
    value->fault_flags = data[1];
    value->uptime_s = rp_get_u16_be(&data[2]);
    value->fw_major = data[4];
    value->fw_minor = data[5];
    value->last_cmd_age_ms = rp_get_u16_be(&data[6]);
    return true;
}

bool rp_pack_line_state(const rp_line_state_t *value,
                        uint8_t out[RP_LINE_STATE_LEN])
{
    if (value == NULL || out == NULL || value->ccd_center > 127u ||
        value->ccd_confidence > 100u ||
        value->line_error < -100 || value->line_error > 100 ||
        value->left_command < -100 || value->left_command > 100 ||
        value->right_command < -100 || value->right_command > 100 ||
        value->rewind_progress > 100u) {
        return false;
    }
    out[0] = value->flags;
    out[1] = value->source;
    out[2] = value->ccd_center;
    out[3] = value->ccd_confidence;
    out[4] = (uint8_t)value->line_error;
    out[5] = value->lf_bits;
    out[6] = (uint8_t)value->left_command;
    out[7] = (uint8_t)value->right_command;
    out[8] = value->rewind_progress;
    out[9] = value->distance_cm;
    return true;
}

bool rp_unpack_line_state(const uint8_t *data,
                          uint8_t len,
                          rp_line_state_t *value)
{
    rp_line_state_t decoded;

    if (data == NULL || value == NULL || len != RP_LINE_STATE_LEN) {
        return false;
    }
    decoded.flags = data[0];
    decoded.source = data[1];
    decoded.ccd_center = data[2];
    decoded.ccd_confidence = data[3];
    decoded.line_error = (int8_t)data[4];
    decoded.lf_bits = data[5];
    decoded.left_command = (int8_t)data[6];
    decoded.right_command = (int8_t)data[7];
    decoded.rewind_progress = data[8];
    decoded.distance_cm = data[9];
    if (decoded.ccd_center > 127u || decoded.ccd_confidence > 100u ||
        decoded.line_error < -100 || decoded.line_error > 100 ||
        decoded.left_command < -100 || decoded.left_command > 100 ||
        decoded.right_command < -100 || decoded.right_command > 100 ||
        decoded.rewind_progress > 100u) {
        return false;
    }
    *value = decoded;
    return true;
}

bool rp_line_has_signal(const rp_line_state_t *state)
{
    const uint8_t ccd = RP_LINE_FLAG_CCD_VALID | RP_LINE_FLAG_CCD_FOUND;
    /* LF04 outputs are active low. 0x0F is four white sensors. */
    return (state->flags & ccd) == ccd || (state->lf_bits & 0x0Fu) != 0x0Fu;
}

bool rp_pack_imu_state(const rp_imu_state_t *value,
                       uint8_t out[RP_IMU_STATE_LEN])
{
    if (value == NULL || out == NULL) {
        return false;
    }
    rp_put_i16_be(&out[0], value->gyro_x_ddps);
    rp_put_i16_be(&out[2], value->gyro_y_ddps);
    rp_put_i16_be(&out[4], value->gyro_z_ddps);
    rp_put_i16_be(&out[6], value->accel_x_mg);
    rp_put_i16_be(&out[8], value->accel_y_mg);
    rp_put_i16_be(&out[10], value->accel_z_mg);
    return true;
}

bool rp_unpack_imu_state(const uint8_t *data,
                         uint8_t len,
                         rp_imu_state_t *value)
{
    if (data == NULL || value == NULL || len != RP_IMU_STATE_LEN) {
        return false;
    }
    value->gyro_x_ddps = rp_get_i16_be(&data[0]);
    value->gyro_y_ddps = rp_get_i16_be(&data[2]);
    value->gyro_z_ddps = rp_get_i16_be(&data[4]);
    value->accel_x_mg = rp_get_i16_be(&data[6]);
    value->accel_y_mg = rp_get_i16_be(&data[8]);
    value->accel_z_mg = rp_get_i16_be(&data[10]);
    return true;
}

bool rp_pack_motor_state(const rp_motor_state_t *value,
                         uint8_t out[RP_MOTOR_STATE_LEN])
{
    if (value == NULL || out == NULL || value->pan_deg > RP_PAN_MAX_DEG ||
        (value->pan_flags & ~(RP_PAN_KNOWN | RP_PAN_MOVING | RP_PAN_BOOT_RETURN)) != 0u ||
        value->left_command < -100 || value->left_command > 100 ||
        value->right_command < -100 || value->right_command > 100) {
        return false;
    }
    rp_put_i16_be(&out[0], value->encoder_delta_a);
    rp_put_i16_be(&out[2], value->encoder_delta_b);
    rp_put_i16_be(&out[4], value->pwm_a);
    rp_put_i16_be(&out[6], value->pwm_b);
    rp_put_i16_be(&out[8], value->target_a_mm_s);
    rp_put_i16_be(&out[10], value->target_b_mm_s);
    rp_put_u16_be(&out[12], value->pan_deg);
    out[14] = value->control_source;
    out[15] = (uint8_t)value->left_command;
    out[16] = (uint8_t)value->right_command;
    out[17] = value->pan_flags;
    return true;
}

bool rp_unpack_motor_state(const uint8_t *data,
                           uint8_t len,
                           rp_motor_state_t *value)
{
    rp_motor_state_t decoded;

    if (data == NULL || value == NULL || len != RP_MOTOR_STATE_LEN) {
        return false;
    }
    decoded.encoder_delta_a = rp_get_i16_be(&data[0]);
    decoded.encoder_delta_b = rp_get_i16_be(&data[2]);
    decoded.pwm_a = rp_get_i16_be(&data[4]);
    decoded.pwm_b = rp_get_i16_be(&data[6]);
    decoded.target_a_mm_s = rp_get_i16_be(&data[8]);
    decoded.target_b_mm_s = rp_get_i16_be(&data[10]);
    decoded.pan_deg = rp_get_u16_be(&data[12]);
    decoded.control_source = data[14];
    decoded.left_command = (int8_t)data[15];
    decoded.right_command = (int8_t)data[16];
    decoded.pan_flags = data[17];
    if (decoded.pan_deg > RP_PAN_MAX_DEG ||
        (decoded.pan_flags & ~(RP_PAN_KNOWN | RP_PAN_MOVING | RP_PAN_BOOT_RETURN)) != 0u ||
        decoded.left_command < -100 || decoded.left_command > 100 ||
        decoded.right_command < -100 || decoded.right_command > 100) {
        return false;
    }
    *value = decoded;
    return true;
}

bool rp_pack_command_ack(const rp_command_ack_t *value,
                         uint8_t out[RP_COMMAND_ACK_LEN])
{
    if (value == NULL || out == NULL) {
        return false;
    }
    out[0] = value->command;
    out[1] = value->status;
    return true;
}

bool rp_unpack_command_ack(const uint8_t *data,
                           uint8_t len,
                           rp_command_ack_t *value)
{
    if (data == NULL || value == NULL || len != RP_COMMAND_ACK_LEN) {
        return false;
    }
    value->command = data[0];
    value->status = data[1];
    return true;
}

/* ------------------------------------------------------------------ params */
static const struct {
    int32_t min_value;
    int32_t max_value;
    int32_t default_value;
} s_param_ranges[RP_PARAM_COUNT] = {
    {5, 40, 30},     /* line base percent */
    {0, 60, 12},     /* line turn gain */
    {0, 5000, 700},  /* speed Kp */
    {0, 5000, 700},  /* speed Ki */
    {50, 350, 350},  /* max wheel speed, mm/s */
};

bool rp_param_range(uint8_t id, int32_t *min_value, int32_t *max_value,
                    int32_t *default_value)
{
    if (id < 1u || id > RP_PARAM_COUNT) {
        return false;
    }
    if (min_value != NULL) {
        *min_value = s_param_ranges[id - 1u].min_value;
    }
    if (max_value != NULL) {
        *max_value = s_param_ranges[id - 1u].max_value;
    }
    if (default_value != NULL) {
        *default_value = s_param_ranges[id - 1u].default_value;
    }
    return true;
}

bool rp_param_valid(uint8_t id, int32_t value)
{
    if (id < 1u || id > RP_PARAM_COUNT) {
        return false;
    }
    return value >= s_param_ranges[id - 1u].min_value &&
           value <= s_param_ranges[id - 1u].max_value;
}

bool rp_pack_param(uint8_t id, int32_t value,
                   uint8_t out[RP_PARAM_FRAME_LEN])
{
    if (out == NULL || !rp_param_valid(id, value)) {
        return false;
    }
    out[0] = id;
    out[1] = (uint8_t)((uint32_t)value >> 24);
    out[2] = (uint8_t)((uint32_t)value >> 16);
    out[3] = (uint8_t)((uint32_t)value >> 8);
    out[4] = (uint8_t)value;
    return true;
}

bool rp_unpack_param(const uint8_t *data, uint8_t len, uint8_t *id,
                     int32_t *value)
{
    uint8_t decoded_id;
    int32_t decoded_value;

    if (data == NULL || len != RP_PARAM_FRAME_LEN) {
        return false;
    }
    decoded_id = data[0];
    decoded_value = (int32_t)(((uint32_t)data[1] << 24) |
                              ((uint32_t)data[2] << 16) |
                              ((uint32_t)data[3] << 8) |
                              (uint32_t)data[4]);
    if (!rp_param_valid(decoded_id, decoded_value)) {
        return false;
    }
    if (id != NULL) {
        *id = decoded_id;
    }
    if (value != NULL) {
        *value = decoded_value;
    }
    return true;
}
