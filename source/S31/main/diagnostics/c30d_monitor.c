#include "c30d_monitor.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static td_c30d_sample_t s_samples[TD_C30D_HISTORY_CAPACITY];
static size_t s_head;
static size_t s_count;
static uint32_t s_last_capture_ms;
static bool s_has_capture;

void td_c30d_monitor_init(void)
{
    memset(s_samples, 0, sizeof(s_samples));
    s_head = 0u;
    s_count = 0u;
    s_last_capture_ms = 0u;
    s_has_capture = false;
}

void td_c30d_monitor_poll(const td_c30d_sample_t *sample, uint32_t now_ms)
{
    td_c30d_sample_t captured;
    size_t index;

    if (sample == NULL) {
        return;
    }

    if (s_has_capture && (uint32_t)(now_ms - s_last_capture_ms) < 1000u) {
        return;
    }

    captured = *sample;
    captured.uptime_ms = now_ms;

    if (s_count < TD_C30D_HISTORY_CAPACITY) {
        index = (s_head + s_count) % TD_C30D_HISTORY_CAPACITY;
        ++s_count;
    } else {
        index = s_head;
        s_head = (s_head + 1u) % TD_C30D_HISTORY_CAPACITY;
    }
    s_samples[index] = captured;
    s_last_capture_ms = now_ms;
    s_has_capture = true;
}

size_t td_c30d_monitor_snapshot(td_c30d_sample_t *out, size_t capacity)
{
    size_t copied;

    copied = s_count < capacity ? s_count : capacity;
    if (out != NULL) {
        size_t start = (s_head + s_count - copied) %
                       TD_C30D_HISTORY_CAPACITY;
        for (size_t i = 0; i < copied; ++i) {
            out[i] = s_samples[(start + i) % TD_C30D_HISTORY_CAPACITY];
        }
    }
    return copied;
}

size_t td_c30d_monitor_export_csv(char *out, size_t capacity)
{
    td_c30d_sample_t samples[TD_C30D_HISTORY_CAPACITY];
    size_t count;
    size_t used;
    int written;

    if (out == NULL || capacity == 0u) {
        return 0u;
    }
    out[0] = '\0';

    count = td_c30d_monitor_snapshot(samples, TD_C30D_HISTORY_CAPACITY);
    if (count == 0u) {
        return 0u;
    }

    written = snprintf(
        out, capacity,
        "time_ms,online,state,faults,left_mm_s,right_mm_s,yaw_cdeg,"
        "distance_cm,battery_mv,line_flags,line_source,ccd_center,"
        "ccd_confidence,line_error,rewind_progress,gyro_x_ddps,"
        "gyro_y_ddps,gyro_z_ddps,accel_x_mg,accel_y_mg,accel_z_mg\r\n");
    if (written < 0 || (size_t)written >= capacity) {
        out[0] = '\0';
        return 0u;
    }
    used = (size_t)written;

    for (size_t i = 0; i < count; ++i) {
        written = snprintf(
            out + used, capacity - used,
            "%lu,%u,%u,%u,%d,%d,%d,%u,%u,%u,%u,%u,%u,%d,%u,"
            "%d,%d,%d,%d,%d,%d\r\n",
            (unsigned long)samples[i].uptime_ms,
            (unsigned int)samples[i].online,
            (unsigned int)samples[i].robot_state,
            (unsigned int)samples[i].fault_flags,
            (int)samples[i].left_speed_mm_s,
            (int)samples[i].right_speed_mm_s,
            (int)samples[i].yaw_cdeg,
            (unsigned int)samples[i].distance_cm,
            (unsigned int)samples[i].battery_mv,
            (unsigned int)samples[i].line_flags,
            (unsigned int)samples[i].line_source,
            (unsigned int)samples[i].ccd_center,
            (unsigned int)samples[i].ccd_confidence,
            (int)samples[i].line_error,
            (unsigned int)samples[i].rewind_progress,
            (int)samples[i].gyro_x_ddps,
            (int)samples[i].gyro_y_ddps,
            (int)samples[i].gyro_z_ddps,
            (int)samples[i].accel_x_mg,
            (int)samples[i].accel_y_mg,
            (int)samples[i].accel_z_mg);
        if (written < 0 || (size_t)written >= capacity - used) {
            out[0] = '\0';
            return 0u;
        }
        used += (size_t)written;
    }
    return used;
}

static bool append_json(char *out, size_t capacity, size_t *used,
                        const char *format, ...)
{
    va_list args;
    int written;

    va_start(args, format);
    written = vsnprintf(out + *used, capacity - *used, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= capacity - *used) {
        out[0] = '\0';
        return false;
    }
    *used += (size_t)written;
    return true;
}

size_t td_c30d_monitor_export_json(const td_c30d_meta_t *meta,
                                   const td_c30d_sample_t *samples,
                                   size_t count,
                                   char *out,
                                   size_t capacity)
{
    size_t used = 0u;
    bool ok;

    if (meta == NULL || out == NULL || capacity == 0u ||
        (count > 0u && samples == NULL)) {
        if (out != NULL && capacity > 0u) {
            out[0] = '\0';
        }
        return 0u;
    }

    out[0] = '\0';
    ok = append_json(
        out, capacity, &used,
        "{\"usb_connected\":%s,\"online\":%s,\"last_rx_age_ms\":%lu,"
        "\"rx_frames\":%lu,\"rx_bytes\":%lu,"
        "\"chassis_frames\":%lu,\"sensor_frames\":%lu,"
        "\"safety_frames\":%lu,\"line_frames\":%lu,"
        "\"imu_frames\":%lu,\"motor_frames\":%lu,\"crc_errors\":%lu,"
        "\"length_errors\":%lu,\"payload_errors\":%lu,"
        "\"frame_age_ms\":{\"chassis\":%lu,\"sensor\":%lu,\"safety\":%lu,"
        "\"line\":%lu,\"imu\":%lu,\"motor\":%lu},",
        meta->usb_connected ? "true" : "false",
        meta->online ? "true" : "false",
        (unsigned long)meta->last_rx_age_ms,
        (unsigned long)meta->rx_frames,
        (unsigned long)meta->rx_bytes,
        (unsigned long)meta->chassis_frames,
        (unsigned long)meta->sensor_frames,
        (unsigned long)meta->safety_frames,
        (unsigned long)meta->line_frames,
        (unsigned long)meta->imu_frames,
        (unsigned long)meta->motor_frames,
        (unsigned long)meta->crc_errors,
        (unsigned long)meta->length_errors,
        (unsigned long)meta->payload_errors,
        (unsigned long)meta->chassis_age_ms,
        (unsigned long)meta->sensor_age_ms,
        (unsigned long)meta->safety_age_ms,
        (unsigned long)meta->line_age_ms,
        (unsigned long)meta->imu_age_ms,
        (unsigned long)meta->motor_age_ms);

    if (ok) {
        ok = append_json(
            out, capacity, &used,
            "\"fw_major\":%u,\"fw_minor\":%u,"
            "\"robot_state\":%u,\"fault_flags\":%u,"
            "\"left_speed_mm_s\":%d,\"right_speed_mm_s\":%d,"
            "\"yaw_cdeg\":%d,\"distance_cm\":%u,\"battery_mv\":%u,"
            "\"line_flags\":%u,\"line_source\":%u,"
            "\"ccd_center\":%u,\"ccd_confidence\":%u,"
            "\"line_error\":%d,\"rewind_progress\":%u,"
            "\"gyro_x_ddps\":%d,\"gyro_y_ddps\":%d,"
            "\"gyro_z_ddps\":%d,\"accel_x_mg\":%d,"
            "\"accel_y_mg\":%d,\"accel_z_mg\":%d,"
            "\"encoder_delta_a\":%d,\"encoder_delta_b\":%d,"
            "\"pwm_a\":%d,\"pwm_b\":%d,"
            "\"target_a_mm_s\":%d,\"target_b_mm_s\":%d,"
            "\"pan_deg\":%u,\"control_source\":%u,"
            "\"left_command\":%d,\"right_command\":%d,"
            "\"c30d_uptime_s\":%u,\"last_cmd_age_ms\":%u,"
            "\"last_cmd\":%u,\"history_seconds\":%u,\"samples\":[",
            (unsigned int)meta->fw_major,
            (unsigned int)meta->fw_minor,
            (unsigned int)meta->robot_state,
            (unsigned int)meta->fault_flags,
            (int)meta->left_speed_mm_s,
            (int)meta->right_speed_mm_s,
            (int)meta->yaw_cdeg,
            (unsigned int)meta->distance_cm,
            (unsigned int)meta->battery_mv,
            (unsigned int)meta->line_flags,
            (unsigned int)meta->line_source,
            (unsigned int)meta->ccd_center,
            (unsigned int)meta->ccd_confidence,
            (int)meta->line_error,
            (unsigned int)meta->rewind_progress,
            (int)meta->gyro_x_ddps,
            (int)meta->gyro_y_ddps,
            (int)meta->gyro_z_ddps,
            (int)meta->accel_x_mg,
            (int)meta->accel_y_mg,
            (int)meta->accel_z_mg,
            (int)meta->encoder_delta_a,
            (int)meta->encoder_delta_b,
            (int)meta->pwm_a,
            (int)meta->pwm_b,
            (int)meta->target_a_mm_s,
            (int)meta->target_b_mm_s,
            (unsigned int)meta->pan_deg,
            (unsigned int)meta->control_source,
            (int)meta->left_command,
            (int)meta->right_command,
            (unsigned int)meta->c30d_uptime_s,
            (unsigned int)meta->last_cmd_age_ms,
            (unsigned int)meta->last_cmd,
            TD_C30D_HISTORY_CAPACITY);
    }

    for (size_t i = 0; ok && i < count; ++i) {
        ok = append_json(
            out, capacity, &used,
            "%s{\"t\":%lu,\"on\":%u,\"state\":%u,\"faults\":%u,"
            "\"left\":%d,\"right\":%d,\"yaw\":%d,"
            "\"distance\":%u,\"battery\":%u,\"line_flags\":%u,"
            "\"line_source\":%u,\"ccd_center\":%u,"
            "\"ccd_confidence\":%u,\"line_error\":%d,"
            "\"rewind_progress\":%u,\"gyro_z_ddps\":%d,"
            "\"accel_z_mg\":%d}",
            i == 0u ? "" : ",",
            (unsigned long)samples[i].uptime_ms,
            (unsigned int)samples[i].online,
            (unsigned int)samples[i].robot_state,
            (unsigned int)samples[i].fault_flags,
            (int)samples[i].left_speed_mm_s,
            (int)samples[i].right_speed_mm_s,
            (int)samples[i].yaw_cdeg,
            (unsigned int)samples[i].distance_cm,
            (unsigned int)samples[i].battery_mv,
            (unsigned int)samples[i].line_flags,
            (unsigned int)samples[i].line_source,
            (unsigned int)samples[i].ccd_center,
            (unsigned int)samples[i].ccd_confidence,
            (int)samples[i].line_error,
            (unsigned int)samples[i].rewind_progress,
            (int)samples[i].gyro_z_ddps,
            (int)samples[i].accel_z_mg);
    }

    if (ok) {
        ok = append_json(out, capacity, &used, "]}");
    }
    return ok ? used : 0u;
}
