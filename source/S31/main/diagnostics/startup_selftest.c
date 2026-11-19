#include "startup_selftest.h"
#include <string.h>
#include "app/app_state.h"
#include "perception/camera_app.h"
#include "agent/agent_service.h"
#include "esp_log.h"
#include "perception/face_service.h"
#include "perception/object_service.h"
#include "audio/voice_player.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "selftest";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static startup_report_t s_report;
static startup_init_t s_init;
static bool s_started;

void startup_selftest_snapshot(startup_report_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_lock);
    *out = s_report;
    portEXIT_CRITICAL(&s_lock);
}

void startup_selftest_log(void)
{
    startup_report_t r;
    startup_selftest_snapshot(&r);
    ESP_LOGI(TAG, "status running=%d ready=%d complete=%d announcing=%d errors=%u attention=%u audio=%s",
             r.running, r.ready, r.complete, r.announcing, r.checks.errors,
             r.checks.attention, esp_err_to_name(r.audio_error));
    if (!r.ready) return;
    for (int m = 0; m < ST_ITEM_COUNT; ++m) {
        ESP_LOGI(TAG, "item %s=%s", st_module_id(m), st_result_id(r.checks.results[m]));
    }
}

static void selftest_task(void *arg)
{
    (void)arg;
    td_status_t before, after;
    td_face_status_t face;
    td_object_status_t objects;
    voice_status_t voice;
    st_inputs_t input = {0};
    st_results_t checks;
    uint32_t started_at = app_now_ms();
    /* Only observe peripherals; never arm motors, clear E-STOP or move the pan. */
    vTaskDelay(pdMS_TO_TICKS(5500));
    /* Model loading has a large temporary memory peak. Wait for initialized
     * services to settle before reading many SPIFFS prompts or judging ready. */
    while ((uint32_t)(app_now_ms() - started_at) < 20000u) {
        face_service_snapshot(&face);
        object_service_snapshot(&objects);
        if ((!s_init.face || face.ready) && (!s_init.objects || objects.ready)) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    app_state_snapshot(&before);
    /* Require each telemetry group to advance within the final half second. */
    vTaskDelay(pdMS_TO_TICKS(500));
    app_state_snapshot(&after);
    face_service_snapshot(&face);
    object_service_snapshot(&objects);
    voice_get_status(&voice);
    input.storage = s_init.storage;
    input.display = s_init.display;
    input.touch = s_init.touch;
    input.keys = s_init.keys;
    input.audio = voice.ready && voice.last_error == ESP_OK && voice.completed > 0u;
    input.camera_init = s_init.camera;
    input.camera_frame = camera_app_has_fresh_frame();
    input.face_init = s_init.face;
    input.face_ready = face.ready;
    input.objects_init = s_init.objects;
    input.objects_ready = objects.ready;
    input.wifi_init = s_init.wifi;
    input.wifi_up = after.sta_connected;
    input.wifi_ap = after.ap_started;
    input.web = after.web_ready;
    input.agent_init = s_init.agent;
    input.agent_configured = agent_service_is_configured();
    input.usb_init = s_init.usb;
    input.chassis_online = after.stm32_online;
    input.sensor_fresh = after.stm32_sensor_frames != before.stm32_sensor_frames;
    input.imu_fresh = after.stm32_imu_frames != before.stm32_imu_frames;
    input.line_fresh = after.stm32_line_frames != before.stm32_line_frames;
    input.motor_fresh = after.stm32_motor_frames != before.stm32_motor_frames;
    input.safety_fresh = after.stm32_safety_frames != before.stm32_safety_frames;
    input.battery_mv = after.sensor.battery_mv;
    input.distance_cm = after.sensor.distance_cm;
    input.faults = after.safety.fault_flags;
    input.line_flags = after.line_state.flags;
    st_evaluate(&input, &checks);
    uint32_t checked_at = app_now_ms();
    portENTER_CRITICAL(&s_lock);
    s_report.checks = checks;
    s_report.ready = true;
    s_report.announcing = voice.ready;
    s_report.checked_at_ms = checked_at;
    portEXIT_CRITICAL(&s_lock);
    startup_selftest_log();
    esp_err_t audio_error = voice.ready ? ESP_OK : ESP_ERR_INVALID_STATE;
    if (voice.ready) {
        for (int m = 0; m < ST_ITEM_COUNT; ++m) {
            st_result_t result = checks.results[m];
            /* One chassis warning replaces eight repetitive downstream messages. */
            if (result == ST_SKIPPED) continue;
            portENTER_CRITICAL(&s_lock);
            s_report.current = (uint8_t)m;
            portEXIT_CRITICAL(&s_lock);
            esp_err_t err = voice_play_pair(st_module_clip(m), st_result_clip(result));
            if (err != ESP_OK) {
                audio_error = err;
                ESP_LOGE(TAG, "announcement %s failed: %s", st_module_id(m), esp_err_to_name(err));
                break;
            }
        }
        if (audio_error == ESP_OK) {
            audio_error = voice_play(checks.attention ? "st_done_warn" : "st_done");
        }
    }
    portENTER_CRITICAL(&s_lock);
    s_report.audio_error = audio_error;
    s_report.announcing = false;
    s_report.running = false;
    s_report.complete = true;
    portEXIT_CRITICAL(&s_lock);
    startup_selftest_log();
    vTaskDelete(NULL);
}

esp_err_t startup_selftest_start(const startup_init_t *init)
{
    if (!init || s_started) return ESP_ERR_INVALID_STATE;
    s_init = *init;
    s_started = true;
    portENTER_CRITICAL(&s_lock);
    s_report.running = true;
    portEXIT_CRITICAL(&s_lock);
    if (xTaskCreatePinnedToCore(selftest_task, "startup_check", 8192, NULL, 2, NULL, 1) != pdPASS) {
        portENTER_CRITICAL(&s_lock);
        s_report.running = false;
        s_report.audio_error = ESP_ERR_NO_MEM;
        portEXIT_CRITICAL(&s_lock);
        s_started = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
