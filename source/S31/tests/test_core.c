#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "diagnostics/c30d_monitor.h"
#include "control/line_follower.h"
#include "perception/rgb565_format.h"
#include "protocol/robot_protocol.h"
#include "protocol/robot_ack.h"

static int s_checks;
static int s_failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        ++s_checks;                                                        \
        if (!(condition)) {                                                \
            ++s_failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
        }                                                                  \
    } while (0)

static void test_ack_latch(void)
{
    rp_ack_latch_t latch = {.command = RP_CMD_DRIVE};
    rp_command_ack_t estop_ack = {RP_CMD_ESTOP, 0u};
    rp_command_ack_t drive_ack = {RP_CMD_DRIVE, 1u};
    CHECK(!rp_ack_offer(&latch, &estop_ack) && !latch.received);
    CHECK(rp_ack_offer(&latch, &drive_ack) && latch.received && latch.status == 1u);
    CHECK(!rp_ack_offer(&latch, &estop_ack) && latch.status == 1u);
    drive_ack.status = 0u;
    CHECK(!rp_ack_offer(&latch, &drive_ack) && latch.status == 1u);
    latch = (rp_ack_latch_t){0};
    CHECK(!rp_ack_offer(&latch, &drive_ack) && !latch.received);
}

static void test_protocol(void)
{
    const uint8_t expected_drive[] = {
        0xAAu, 0x55u, 0x02u, 0x10u, 0x14u, 0xECu, 0x07u
    };
    rp_drive_cmd_t drive = {20, -20};
    rp_drive_cmd_t decoded_drive;
    rp_actuator_cmd_t actuator = {RP_PAN_MAX_DEG, RP_PUSH_STOP};
    rp_actuator_cmd_t decoded_actuator;
    rp_line_cmd_t line = {1, 30};
    rp_line_cmd_t decoded_line;
    rp_chassis_state_t chassis = {-123, 456, -17999};
    rp_chassis_state_t decoded_chassis;
    rp_sensor_state_t sensor = {37, 0x0A, 0x81, 11234};
    rp_sensor_state_t decoded_sensor;
    rp_safety_state_t safety = {
        RP_ROBOT_FAULT,
        RP_FAULT_IMU_OFFLINE | RP_FAULT_ULTRA_TIMEOUT,
        1234, 2, 1, 88,
    };
    rp_safety_state_t decoded_safety;
    rp_line_state_t line_state = {
        RP_LINE_FLAG_ACTIVE | RP_LINE_FLAG_CCD_FOUND |
            RP_LINE_FLAG_CCD_VALID,
        3u, 71u, 86u, -18, 0x0D, 28, 12, 0u, 77u,
    };
    rp_line_state_t decoded_line_state;
    rp_imu_state_t imu_state = {
        -123, 456, 789, -1000, 25, 998,
    };
    rp_imu_state_t decoded_imu_state;
    rp_motor_state_t motor_state = {
        -123, 456, 7890, -7890, 250, -250, 517u, 3u, 18, -12,
        RP_PAN_KNOWN | RP_PAN_MOVING,
    };
    rp_motor_state_t decoded_motor_state;
    uint8_t payload[RP_MAX_PAYLOAD];
    uint8_t frame[RP_MAX_FRAME];
    uint8_t decoded_payload[RP_MAX_PAYLOAD];
    uint8_t cmd = 0;
    uint8_t len = 0;
    rp_parser_t parser;

    CHECK(rp_pack_drive(&drive, payload));
    size_t frame_len = rp_encode_frame(
        RP_CMD_DRIVE, payload, RP_DRIVE_LEN, frame, sizeof(frame));
    CHECK(frame_len == sizeof(expected_drive));
    CHECK(memcmp(frame, expected_drive, sizeof(expected_drive)) == 0);

    rp_parser_init(&parser);
    for (size_t i = 0; i < frame_len; ++i) {
        rp_parse_result_t result = rp_parser_feed(
            &parser, frame[i], &cmd, decoded_payload, &len);
        if (i + 1u == frame_len) {
            CHECK(result == RP_PARSE_FRAME);
            CHECK(cmd == RP_CMD_DRIVE);
            CHECK(len == RP_DRIVE_LEN);
        }
    }
    CHECK(rp_unpack_drive(decoded_payload, len, &decoded_drive));
    CHECK(decoded_drive.left_percent == drive.left_percent);
    CHECK(decoded_drive.right_percent == drive.right_percent);
    CHECK(!rp_unpack_drive(payload, RP_DRIVE_LEN - 1u, &decoded_drive));

    {
        /* v3.9.8 回归：解包失败时不得改写调用者的输出变量 */
        const uint8_t out_of_range[RP_DRIVE_LEN] = {101u, 0u};
        const uint8_t limits[RP_DRIVE_LEN] = {100u, (uint8_t)-100};
        rp_drive_cmd_t kept = {10, 20};
        const rp_drive_cmd_t before = kept;

        CHECK(!rp_unpack_drive(out_of_range, RP_DRIVE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);
        CHECK(!rp_unpack_drive(out_of_range, RP_DRIVE_LEN - 1u, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);
        CHECK(!rp_unpack_drive(NULL, RP_DRIVE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        CHECK(rp_unpack_drive(limits, RP_DRIVE_LEN, &kept));
        CHECK(kept.left_percent == 100);
        CHECK(kept.right_percent == -100);
    }

    CHECK(rp_pack_actuator(&actuator, payload));
    CHECK(rp_unpack_actuator(payload, RP_ACTUATOR_LEN,
                             &decoded_actuator));
    CHECK(decoded_actuator.pan_deg == actuator.pan_deg);
    CHECK(decoded_actuator.push_action == RP_PUSH_STOP);
    CHECK(payload[0] == 2u && payload[1] == 28u);
    CHECK(!rp_unpack_actuator(payload, 2u, &decoded_actuator));
    ++payload[1];
    CHECK(!rp_unpack_actuator(payload, RP_ACTUATOR_LEN, &decoded_actuator));
    CHECK(decoded_actuator.pan_deg == RP_PAN_MAX_DEG);

    CHECK(rp_pack_line_mode(&line, payload));
    CHECK(rp_unpack_line_mode(payload, RP_LINE_LEN, &decoded_line));
    CHECK(decoded_line.enable == 1u);
    CHECK(decoded_line.speed_percent == 30u);
    line.speed_percent = 4u;
    CHECK(!rp_pack_line_mode(&line, payload));

    CHECK(rp_pack_chassis_state(&chassis, payload));
    CHECK(rp_unpack_chassis_state(payload, RP_CHASSIS_STATE_LEN,
                                  &decoded_chassis));
    CHECK(decoded_chassis.left_speed_mm_s == chassis.left_speed_mm_s);
    CHECK(decoded_chassis.right_speed_mm_s == chassis.right_speed_mm_s);
    CHECK(decoded_chassis.yaw_cdeg == chassis.yaw_cdeg);

    CHECK(rp_pack_sensor_state(&sensor, payload));
    CHECK(rp_unpack_sensor_state(payload, RP_SENSOR_STATE_LEN,
                                 &decoded_sensor));
    CHECK(decoded_sensor.distance_cm == sensor.distance_cm);
    CHECK(decoded_sensor.push_top_limit == sensor.push_top_limit);
    CHECK(decoded_sensor.push_bottom_limit == sensor.push_bottom_limit);
    CHECK(decoded_sensor.battery_mv == sensor.battery_mv);

    CHECK(rp_pack_safety_state(&safety, payload));
    CHECK(rp_unpack_safety_state(payload, RP_SAFETY_STATE_LEN,
                                 &decoded_safety));
    CHECK(decoded_safety.robot_state == safety.robot_state);
    CHECK(decoded_safety.fault_flags == safety.fault_flags);
    CHECK(decoded_safety.uptime_s == safety.uptime_s);
    CHECK(decoded_safety.fw_major == safety.fw_major);
    CHECK(decoded_safety.fw_minor == safety.fw_minor);
    CHECK(decoded_safety.last_cmd_age_ms == safety.last_cmd_age_ms);

    CHECK(rp_pack_line_state(&line_state, payload));
    CHECK(rp_unpack_line_state(payload, RP_LINE_STATE_LEN,
                               &decoded_line_state));
    CHECK(decoded_line_state.flags == line_state.flags);
    CHECK(decoded_line_state.source == line_state.source);
    CHECK(decoded_line_state.ccd_center == line_state.ccd_center);
    CHECK(decoded_line_state.ccd_confidence ==
          line_state.ccd_confidence);
    CHECK(decoded_line_state.line_error == line_state.line_error);
    CHECK(decoded_line_state.lf_bits == line_state.lf_bits);
    CHECK(decoded_line_state.left_command == line_state.left_command);
    CHECK(decoded_line_state.right_command == line_state.right_command);
    CHECK(decoded_line_state.rewind_progress ==
          line_state.rewind_progress);
    CHECK(decoded_line_state.distance_cm == line_state.distance_cm);

    {
        /* v3.9.8 回归：非法 line_state 不得留下半截改写 */
        uint8_t bad[RP_LINE_STATE_LEN];
        rp_line_state_t kept = line_state;
        const rp_line_state_t before = kept;

        memcpy(bad, payload, sizeof(bad));
        bad[2] = 200u; /* ccd_center 越界 */
        CHECK(!rp_unpack_line_state(bad, RP_LINE_STATE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        memcpy(bad, payload, sizeof(bad));
        bad[3] = 101u; /* ccd_confidence 越界 */
        CHECK(!rp_unpack_line_state(bad, RP_LINE_STATE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        memcpy(bad, payload, sizeof(bad));
        bad[6] = 101u; /* left_command 越界 */
        CHECK(!rp_unpack_line_state(bad, RP_LINE_STATE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        CHECK(!rp_unpack_line_state(payload, RP_LINE_STATE_LEN - 1u, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);
    }

    CHECK(rp_pack_imu_state(&imu_state, payload));
    CHECK(rp_unpack_imu_state(payload, RP_IMU_STATE_LEN,
                              &decoded_imu_state));
    CHECK(decoded_imu_state.gyro_x_ddps == imu_state.gyro_x_ddps);
    CHECK(decoded_imu_state.gyro_y_ddps == imu_state.gyro_y_ddps);
    CHECK(decoded_imu_state.gyro_z_ddps == imu_state.gyro_z_ddps);
    CHECK(decoded_imu_state.accel_x_mg == imu_state.accel_x_mg);
    CHECK(decoded_imu_state.accel_y_mg == imu_state.accel_y_mg);
    CHECK(decoded_imu_state.accel_z_mg == imu_state.accel_z_mg);

    CHECK(rp_pack_motor_state(&motor_state, payload));
    CHECK(rp_unpack_motor_state(payload, RP_MOTOR_STATE_LEN,
                                &decoded_motor_state));
    CHECK(decoded_motor_state.encoder_delta_a ==
          motor_state.encoder_delta_a);
    CHECK(decoded_motor_state.encoder_delta_b ==
          motor_state.encoder_delta_b);
    CHECK(decoded_motor_state.pwm_a == motor_state.pwm_a);
    CHECK(decoded_motor_state.pwm_b == motor_state.pwm_b);
    CHECK(decoded_motor_state.target_a_mm_s ==
          motor_state.target_a_mm_s);
    CHECK(decoded_motor_state.target_b_mm_s ==
          motor_state.target_b_mm_s);
    CHECK(decoded_motor_state.pan_deg == motor_state.pan_deg);
    CHECK(decoded_motor_state.control_source ==
          motor_state.control_source);
    CHECK(decoded_motor_state.left_command ==
          motor_state.left_command);
    CHECK(decoded_motor_state.right_command ==
          motor_state.right_command);

    {
        /* v3.9.8 回归：非法 motor_state 不得留下半截改写 */
        uint8_t bad[RP_MOTOR_STATE_LEN];
        rp_motor_state_t kept = motor_state;
        const rp_motor_state_t before = kept;
        const uint8_t limit_bytes[RP_MOTOR_STATE_LEN] = {
            0u, 0u, 0u, 0u, 0u, 0u,
            0u, 0u, 0u, 0u, 0u, 0u,
            2u, 28u, 0u, 100u, (uint8_t)-100, RP_PAN_KNOWN,
        };

        memcpy(bad, payload, sizeof(bad));
        bad[12] = 3u; /* pan_deg 越界 */
        CHECK(!rp_unpack_motor_state(bad, RP_MOTOR_STATE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        memcpy(bad, payload, sizeof(bad));
        bad[15] = 101u; /* left_command 越界 */
        CHECK(!rp_unpack_motor_state(bad, RP_MOTOR_STATE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        memcpy(bad, payload, sizeof(bad));
        bad[16] = (uint8_t)-101; /* right_command 越界 */
        CHECK(!rp_unpack_motor_state(bad, RP_MOTOR_STATE_LEN, &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        CHECK(!rp_unpack_motor_state(payload, RP_MOTOR_STATE_LEN - 1u,
                                     &kept));
        CHECK(memcmp(&kept, &before, sizeof(kept)) == 0);

        /* 边界值仍应成功 */
        CHECK(rp_unpack_motor_state(limit_bytes, RP_MOTOR_STATE_LEN, &kept));
        CHECK(kept.pan_deg == RP_PAN_MAX_DEG);
        CHECK(kept.pan_flags == RP_PAN_KNOWN);
        CHECK(kept.left_command == 100);
        CHECK(kept.right_command == -100);
    }

    {
        const rp_command_ack_t ack = {
            .command = RP_CMD_DRIVE,
            .status = 0u,
        };
        const uint8_t expected_ack[RP_COMMAND_ACK_LEN] = {
            RP_CMD_DRIVE, 0u,
        };
        rp_command_ack_t decoded_ack;

        CHECK(rp_pack_command_ack(&ack, payload));
        CHECK(memcmp(payload, expected_ack, sizeof(expected_ack)) == 0);
        CHECK(rp_unpack_command_ack(payload, RP_COMMAND_ACK_LEN,
                                    &decoded_ack));
        CHECK(decoded_ack.command == RP_CMD_DRIVE);
        CHECK(decoded_ack.status == 0u);
        CHECK(!rp_unpack_command_ack(payload, RP_COMMAND_ACK_LEN - 1u,
                                     &decoded_ack));
    }

    {
        /* v3.10.0: parameter frames share the validate-before-write contract. */
        uint8_t frame[RP_PARAM_FRAME_LEN];
        uint8_t id = 9u;
        int32_t value = 12345;
        int32_t min_value = 0;
        int32_t max_value = 0;
        int32_t default_value = 0;

        CHECK(rp_param_range(RP_PARAM_LINE_BASE_PERCENT, &min_value,
                             &max_value, &default_value));
        CHECK(min_value == 5 && max_value == 40 && default_value == 30);
        CHECK(rp_param_valid(RP_PARAM_LINE_BASE_PERCENT, 30));
        CHECK(!rp_param_valid(RP_PARAM_LINE_BASE_PERCENT, 41));
        CHECK(!rp_param_valid(0u, 30));
        CHECK(!rp_param_range(99u, NULL, NULL, NULL));

        CHECK(rp_pack_param(RP_PARAM_LINE_TURN_GAIN, 25, frame));
        CHECK(frame[0] == RP_PARAM_LINE_TURN_GAIN);
        CHECK(frame[1] == 0u && frame[2] == 0u && frame[3] == 0u &&
              frame[4] == 25u);
        CHECK(rp_unpack_param(frame, RP_PARAM_FRAME_LEN, &id, &value));
        CHECK(id == RP_PARAM_LINE_TURN_GAIN && value == 25);
        CHECK(!rp_pack_param(RP_PARAM_LINE_TURN_GAIN, 61, frame));

        id = 9u;
        value = 12345;
        frame[0] = RP_PARAM_SPEED_KP;
        frame[1] = 0u;
        frame[2] = 0u;
        frame[3] = 0x10u; /* 4096, inside 0..5000 */
        frame[4] = 0x00u;
        CHECK(!rp_unpack_param(frame, RP_PARAM_FRAME_LEN - 1u, &id, &value));
        CHECK(id == 9u && value == 12345);
        CHECK(rp_unpack_param(frame, RP_PARAM_FRAME_LEN, &id, &value));
        CHECK(id == RP_PARAM_SPEED_KP && value == 4096);
        frame[3] = 0x20u; /* 8192 outside 0..5000 */
        CHECK(!rp_unpack_param(frame, RP_PARAM_FRAME_LEN, &id, &value));
        CHECK(id == RP_PARAM_SPEED_KP && value == 4096);
    }

    frame[frame_len - 1u] ^= 1u;
    rp_parser_init(&parser);
    for (size_t i = 0; i < frame_len; ++i) {
        rp_parse_result_t result = rp_parser_feed(
            &parser, frame[i], &cmd, decoded_payload, &len);
        if (i + 1u == frame_len) {
            CHECK(result == RP_PARSE_CRC_ERROR);
        }
    }
}

static void draw_line(uint8_t *image, int center_x)
{
    for (int y = 120; y < 240; ++y) {
        for (int x = center_x - 5; x <= center_x + 5; ++x) {
            image[y * 320 + x] = 0u;
        }
    }
}

static void test_line_follower(void)
{
    uint8_t image[320u * 240u];
    line_follow_config_t config;
    line_follow_result_t result;

    line_follow_default_config(&config);
    config.sample_step = 1u;
    config.min_dark_pixels = 100u;

    memset(image, 255, sizeof(image));
    draw_line(image, 160);
    CHECK(line_follow_compute_gray(image, 320u, 320u, 240u,
                                   &config, &result));
    CHECK(result.line_found);
    CHECK(result.centroid_x == 160u);
    CHECK(result.error >= -1 && result.error <= 1);

    memset(image, 255, sizeof(image));
    draw_line(image, 240);
    CHECK(line_follow_compute_gray(image, 320u, 320u, 240u,
                                   &config, &result));
    CHECK(result.line_found);
    CHECK(result.error >= 49 && result.error <= 51);

    memset(image, 255, sizeof(image));
    CHECK(line_follow_compute_gray(image, 320u, 320u, 240u,
                                   &config, &result));
    CHECK(!result.line_found);
    CHECK(!line_follow_compute_gray(NULL, 320u, 320u, 240u,
                                    &config, &result));
}

static void test_rgb565_endianness(void)
{
    const uint8_t bytes[] = {0x49u, 0x4Au};
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    uint16_t pixel = td_rgb565_read_le(bytes);
    CHECK(pixel == 0x4A49u);
    td_rgb565_to_rgb8(pixel, &red, &green, &blue);
    CHECK(red >= 72u && red <= 76u);
    CHECK(green >= 70u && green <= 76u);
    CHECK(blue >= 72u && blue <= 76u);
}

static void test_c30d_monitor_captures_at_one_hz(void)
{
    td_c30d_sample_t current = {0};
    td_c30d_sample_t samples[4];
    size_t count;

    current.online = 1u;
    current.robot_state = RP_ROBOT_ACTIVE;
    current.left_speed_mm_s = -123;
    current.right_speed_mm_s = 456;
    current.yaw_cdeg = -1799;
    current.distance_cm = 37;
    current.battery_mv = 11234;
    current.line_flags = 0x13u;
    current.line_source = 3u;
    current.ccd_center = 71u;
    current.ccd_confidence = 86u;
    current.line_error = -18;
    current.rewind_progress = 42u;
    current.gyro_x_ddps = -123;
    current.gyro_y_ddps = 456;
    current.gyro_z_ddps = 789;
    current.accel_x_mg = -1000;
    current.accel_y_mg = 25;
    current.accel_z_mg = 998;

    td_c30d_monitor_init();
    td_c30d_monitor_poll(&current, 0u);
    td_c30d_monitor_poll(&current, 999u);

    current.left_speed_mm_s = 88;
    td_c30d_monitor_poll(&current, 1000u);

    count = td_c30d_monitor_snapshot(samples, 4u);
    CHECK(count == 2u);
    CHECK(samples[0].uptime_ms == 0u);
    CHECK(samples[0].left_speed_mm_s == -123);
    CHECK(samples[0].right_speed_mm_s == 456);
    CHECK(samples[0].yaw_cdeg == -1799);
    CHECK(samples[0].distance_cm == 37u);
    CHECK(samples[0].battery_mv == 11234u);
    CHECK(samples[0].line_flags == 0x13u);
    CHECK(samples[0].line_source == 3u);
    CHECK(samples[0].ccd_center == 71u);
    CHECK(samples[0].ccd_confidence == 86u);
    CHECK(samples[0].line_error == -18);
    CHECK(samples[0].rewind_progress == 42u);
    CHECK(samples[0].gyro_z_ddps == 789);
    CHECK(samples[0].accel_z_mg == 998);
    CHECK(samples[0].online == 1u);
    CHECK(samples[1].uptime_ms == 1000u);
    CHECK(samples[1].left_speed_mm_s == 88);
}

static void test_c30d_monitor_exports_csv(void)
{
    const char *expected =
        "time_ms,online,state,faults,left_mm_s,right_mm_s,yaw_cdeg,"
        "distance_cm,battery_mv,line_flags,line_source,ccd_center,"
        "ccd_confidence,line_error,rewind_progress,gyro_x_ddps,"
        "gyro_y_ddps,gyro_z_ddps,accel_x_mg,accel_y_mg,accel_z_mg\r\n"
        "0,1,1,0,-123,456,-1799,37,11234,19,3,71,86,-18,42,"
        "-123,456,789,-1000,25,998\r\n";
    td_c30d_sample_t current = {0};
    char csv[512];
    char small[8] = "keep";
    size_t written;

    current.online = 1u;
    current.robot_state = RP_ROBOT_ACTIVE;
    current.left_speed_mm_s = -123;
    current.right_speed_mm_s = 456;
    current.yaw_cdeg = -1799;
    current.distance_cm = 37;
    current.battery_mv = 11234;
    current.line_flags = 0x13u;
    current.line_source = 3u;
    current.ccd_center = 71u;
    current.ccd_confidence = 86u;
    current.line_error = -18;
    current.rewind_progress = 42u;
    current.gyro_x_ddps = -123;
    current.gyro_y_ddps = 456;
    current.gyro_z_ddps = 789;
    current.accel_x_mg = -1000;
    current.accel_y_mg = 25;
    current.accel_z_mg = 998;

    td_c30d_monitor_init();
    td_c30d_monitor_poll(&current, 0u);
    written = td_c30d_monitor_export_csv(csv, sizeof(csv));
    CHECK(written == strlen(expected));
    CHECK(strcmp(csv, expected) == 0);
    CHECK(td_c30d_monitor_export_csv(small, sizeof(small)) == 0u);
    CHECK(small[0] == '\0');
}

static void test_c30d_monitor_keeps_latest_minute_after_wrap(void)
{
    td_c30d_sample_t current = {0};
    td_c30d_sample_t samples[64];
    size_t count;

    td_c30d_monitor_init();
    for (int i = 0; i <= (int)TD_C30D_HISTORY_CAPACITY; ++i) {
        current.left_speed_mm_s = (int16_t)i;
        td_c30d_monitor_poll(&current, (uint32_t)i * 1000u);
    }

    count = td_c30d_monitor_snapshot(samples, 64u);
    CHECK(count == TD_C30D_HISTORY_CAPACITY);
    CHECK(samples[0].left_speed_mm_s == 1);
    CHECK(samples[TD_C30D_HISTORY_CAPACITY - 1u].left_speed_mm_s == 60);

    count = td_c30d_monitor_snapshot(samples, 3u);
    CHECK(count == 3u);
    CHECK(samples[0].left_speed_mm_s == 58);
    CHECK(samples[2].left_speed_mm_s == 60);
    CHECK(td_c30d_monitor_snapshot(NULL, 7u) == 7u);
    CHECK(td_c30d_monitor_snapshot(NULL, 0u) == 0u);
}

static void test_c30d_monitor_exports_diagnostics_json(void)
{
    td_c30d_meta_t meta = {
        .usb_connected = 1u,
        .online = 1u,
        .last_rx_age_ms = 123u,
        .rx_frames = 9u,
        .rx_bytes = 145u,
        .chassis_frames = 3u,
        .sensor_frames = 3u,
        .safety_frames = 3u,
        .crc_errors = 1u,
        .length_errors = 2u,
        .fw_major = 3u,
        .fw_minor = 5u,
        .robot_state = RP_ROBOT_ACTIVE,
        .fault_flags = RP_FAULT_LOW_BATTERY,
        .left_speed_mm_s = -12,
        .right_speed_mm_s = 34,
        .yaw_cdeg = -1799,
        .distance_cm = 37u,
        .battery_mv = 11234u,
        .c30d_uptime_s = 456u,
        .last_cmd_age_ms = 88u,
        .last_cmd = RP_CMD_CHASSIS_STATE,
        .line_flags = 0x13u,
        .line_source = 3u,
        .ccd_center = 71u,
        .ccd_confidence = 86u,
        .line_error = -18,
        .rewind_progress = 42u,
        .gyro_x_ddps = -123,
        .gyro_y_ddps = 456,
        .gyro_z_ddps = 789,
        .accel_x_mg = -1000,
        .accel_y_mg = 25,
        .accel_z_mg = 998,
    };
    td_c30d_sample_t sample = {
        .uptime_ms = 1000u,
        .left_speed_mm_s = 12,
        .right_speed_mm_s = -34,
        .yaw_cdeg = 900,
        .distance_cm = 20u,
        .battery_mv = 12000u,
        .line_flags = 0x13u,
        .line_source = 3u,
        .ccd_center = 71u,
        .ccd_confidence = 86u,
        .line_error = -18,
        .rewind_progress = 42u,
        .gyro_x_ddps = -123,
        .gyro_y_ddps = 456,
        .gyro_z_ddps = 789,
        .accel_x_mg = -1000,
        .accel_y_mg = 25,
        .accel_z_mg = 998,
        .robot_state = RP_ROBOT_ACTIVE,
        .fault_flags = 0u,
        .online = 1u,
    };
    char json[4096];
    char small[8] = "keep";
    size_t written;

    written = td_c30d_monitor_export_json(
        &meta, &sample, 1u, json, sizeof(json));
    CHECK(written == strlen(json));
    CHECK(strstr(json, "\"usb_connected\":true") != NULL);
    CHECK(strstr(json, "\"online\":true") != NULL);
    CHECK(strstr(json, "\"last_rx_age_ms\":123") != NULL);
    CHECK(strstr(json, "\"rx_frames\":9") != NULL);
    CHECK(strstr(json, "\"chassis_frames\":3") != NULL);
    CHECK(strstr(json, "\"crc_errors\":1") != NULL);
    CHECK(strstr(json, "\"fw_major\":3") != NULL);
    CHECK(strstr(json, "\"left_speed_mm_s\":-12") != NULL);
    CHECK(strstr(json, "\"battery_mv\":11234") != NULL);
    CHECK(strstr(json, "\"line_flags\":19") != NULL);
    CHECK(strstr(json, "\"ccd_center\":71") != NULL);
    CHECK(strstr(json, "\"line_error\":-18") != NULL);
    CHECK(strstr(json, "\"gyro_z_ddps\":789") != NULL);
    CHECK(strstr(json, "\"accel_z_mg\":998") != NULL);
    CHECK(strstr(json, "\"last_cmd\":32") != NULL);
    CHECK(strstr(json, "\"samples\":[{\"t\":1000,\"on\":1") != NULL);
    CHECK(td_c30d_monitor_export_json(
              &meta, &sample, 1u, small, sizeof(small)) == 0u);
    CHECK(small[0] == '\0');
}

static void test_c30d_diagnostics_json_fits_full_history_worst_case(void)
{
    td_c30d_meta_t meta = {0};
    td_c30d_sample_t samples[TD_C30D_HISTORY_CAPACITY];
    char json[TD_C30D_JSON_CAPACITY];

    meta.usb_connected = 1u;
    meta.online = 1u;
    meta.rx_frames = UINT32_MAX;
    meta.rx_bytes = UINT32_MAX;
    for (size_t i = 0u; i < TD_C30D_HISTORY_CAPACITY; ++i) {
        samples[i].uptime_ms = UINT32_MAX;
        samples[i].online = 1u;
        samples[i].robot_state = RP_ROBOT_FAULT;
        samples[i].fault_flags = UINT8_MAX;
        samples[i].left_speed_mm_s = INT16_MIN;
        samples[i].right_speed_mm_s = INT16_MAX;
        samples[i].yaw_cdeg = INT16_MIN;
        samples[i].distance_cm = UINT16_MAX;
        samples[i].battery_mv = UINT16_MAX;
        samples[i].line_flags = UINT8_MAX;
        samples[i].line_source = UINT8_MAX;
        samples[i].ccd_center = 127u;
        samples[i].ccd_confidence = 100u;
        samples[i].line_error = INT8_MIN;
        samples[i].rewind_progress = 100u;
        samples[i].gyro_z_ddps = INT16_MIN;
        samples[i].accel_z_mg = INT16_MIN;
    }
    CHECK(td_c30d_monitor_export_json(
              &meta, samples, TD_C30D_HISTORY_CAPACITY,
              json, sizeof(json)) != 0u);
}

int main(void)
{
    test_ack_latch();
    test_protocol();
    test_line_follower();
    test_rgb565_endianness();
    test_c30d_monitor_captures_at_one_hz();
    test_c30d_monitor_exports_csv();
    test_c30d_monitor_keeps_latest_minute_after_wrap();
    test_c30d_monitor_exports_diagnostics_json();
    test_c30d_diagnostics_json_fits_full_history_worst_case();
    printf("checks=%d failures=%d\n", s_checks, s_failures);
    return s_failures == 0 ? 0 : 1;
}
