#include <stdio.h>
#include <string.h>
#include "diagnostics/selftest_policy.h"
#include "protocol/robot_protocol.h"
static int checks, failures;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)

static st_inputs_t healthy(void)
{
    st_inputs_t i = {0};
    i.storage = i.display = i.touch = i.keys = i.audio = true;
    i.camera_init = i.camera_frame = true;
    i.face_init = i.face_ready = i.objects_init = i.objects_ready = true;
    i.wifi_init = i.wifi_up = i.web = i.agent_init = i.agent_configured = true;
    i.usb_init = i.chassis_online = true;
    i.sensor_fresh = i.imu_fresh = i.line_fresh = i.motor_fresh = i.safety_fresh = true;
    i.battery_mv = 12300;
    i.distance_cm = 100;
    i.line_flags = RP_LINE_FLAG_CCD_VALID | RP_LINE_FLAG_CCD_FOUND;
    return i;
}

int main(void)
{
    st_inputs_t i = healthy();
    st_results_t r;
    st_evaluate(&i, &r);
    CHECK(r.errors == 0 && r.attention == 2);
    CHECK(r.results[ST_MOTORS] == ST_MANUAL);
    CHECK(r.results[ST_INFRARED] == ST_MANUAL);
    CHECK(r.results[ST_CAMERA] == ST_DATA_OK);
    CHECK(r.results[ST_CCD] == ST_SIGNAL_OK);
    CHECK(r.results[ST_PAN] == ST_INIT_OK);
    CHECK(r.results[ST_IMU] == ST_DATA_OK);

    i.camera_frame = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_CAMERA] == ST_NO_DATA);
    i.camera_init = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_CAMERA] == ST_FAULT);

    i = healthy(); i.chassis_online = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_CHASSIS] == ST_DISCONNECTED);
    for (int m = ST_BATTERY; m < ST_ITEM_COUNT; ++m) CHECK(r.results[m] == ST_SKIPPED);
    i.usb_init = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_CHASSIS] == ST_FAULT);

    i = healthy(); i.distance_cm = 0;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_ULTRA] == ST_NO_ECHO);
    CHECK(!st_is_error(r.results[ST_ULTRA]) && !st_needs_attention(r.results[ST_ULTRA]));
    CHECK(r.errors == 0);
    i.faults = RP_FAULT_ULTRA_TIMEOUT;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_ULTRA] == ST_NO_ECHO);
    CHECK(r.results[ST_SAFETY] == ST_INIT_OK);
    CHECK(r.errors == 0);
    i.faults = 0;
    i.distance_cm = UINT16_MAX;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_ULTRA] == ST_NO_ECHO);
    i.distance_cm = 3;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_ULTRA] == ST_DATA_OK);
    i.sensor_fresh = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_ULTRA] == ST_NO_DATA && r.results[ST_BATTERY] == ST_NO_DATA);

    i = healthy(); i.battery_mv = 0;
    st_evaluate(&i, &r); CHECK(r.results[ST_BATTERY] == ST_NO_DATA);
    i.battery_mv = UINT16_MAX;
    st_evaluate(&i, &r); CHECK(r.results[ST_BATTERY] == ST_NO_DATA);
    i.battery_mv = 9500;
    st_evaluate(&i, &r); CHECK(r.results[ST_BATTERY] == ST_LOW_BATTERY);
    i.battery_mv = 12300; i.faults = RP_FAULT_LOW_BATTERY;
    st_evaluate(&i, &r); CHECK(r.results[ST_BATTERY] == ST_LOW_BATTERY);

    i = healthy(); i.faults = RP_FAULT_ESTOP | RP_FAULT_GIMBAL_UNHOMED | RP_FAULT_IMU_OFFLINE;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_SAFETY] == ST_ESTOP);
    CHECK(r.results[ST_PAN] == ST_CALIBRATE);
    CHECK(r.results[ST_IMU] == ST_FAULT);
    i.safety_fresh = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_SAFETY] == ST_NO_DATA);
    CHECK(r.results[ST_PAN] == ST_NO_DATA);
    CHECK(r.results[ST_IMU] == ST_NO_DATA);

    i = healthy(); i.line_flags = RP_LINE_FLAG_CCD_VALID;
    st_evaluate(&i, &r); CHECK(r.results[ST_CCD] == ST_MANUAL);
    i.line_flags = 0;
    st_evaluate(&i, &r); CHECK(r.results[ST_CCD] == ST_NO_DATA);

    i = healthy(); i.wifi_up = false; i.wifi_ap = true;
    st_evaluate(&i, &r); CHECK(r.results[ST_WIFI] == ST_NET_AP);
    i.wifi_ap = false;
    st_evaluate(&i, &r); CHECK(r.results[ST_WIFI] == ST_NET_WAIT);
    i.wifi_init = false;
    st_evaluate(&i, &r); CHECK(r.results[ST_WIFI] == ST_FAULT);

    i = healthy(); i.face_ready = false; i.objects_ready = false; i.agent_configured = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_FACE] == ST_NOT_READY);
    CHECK(r.results[ST_OBJECTS] == ST_NOT_READY);
    CHECK(r.results[ST_AGENT] == ST_UNCONFIGURED);
    i.face_init = i.objects_init = i.agent_init = false;
    st_evaluate(&i, &r);
    CHECK(r.results[ST_FACE] == ST_FAULT);
    CHECK(r.results[ST_OBJECTS] == ST_FAULT);
    CHECK(r.results[ST_AGENT] == ST_FAULT);
    i.audio = false;
    st_evaluate(&i, &r); CHECK(r.results[ST_AUDIO] == ST_FAULT);

    for (int m = 0; m < ST_ITEM_COUNT; ++m) {
        CHECK(st_module_id(m) != NULL && strlen(st_module_id(m)) > 0);
        CHECK(st_module_clip(m) != NULL && strlen(st_module_clip(m)) < 24);
    }
    for (int state = ST_INIT_OK; state <= ST_SKIPPED; ++state) {
        CHECK(st_result_clip(state) != NULL && strlen(st_result_clip(state)) < 24);
    }
    CHECK(st_module_clip((st_module_t)-1) == NULL);
    CHECK(st_result_clip((st_result_t)-1) == NULL);
    st_evaluate(NULL, &r); CHECK(r.attention == ST_ITEM_COUNT);
    printf("selftest checks=%d failures=%d\n", checks, failures);
    return failures ? 1 : 0;
}
