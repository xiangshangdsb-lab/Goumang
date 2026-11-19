#include "face_service.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <list>
#include <new>
#include <vector>

#include "app/app_config.h"
#include "audio/voice_player.h"
#include "dl_image_define.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "human_face_detect.hpp"
#include "human_face_recognition.hpp"

namespace {

constexpr char TAG[] = "face";
constexpr char DB_PATH[] = "/storage/faces.db";
constexpr uint32_t CAPTURE_INTERVAL_US = 400000;
constexpr int64_t ENROLL_TIMEOUT_US = 8000000;

enum class FaceCommand : uint8_t {
    NONE,
    ENROLL,
    DELETE_LAST,
    CLEAR_ALL,
};

HumanFaceDetect *s_detect;
HumanFaceRecognizer *s_recognizer;
SemaphoreHandle_t s_frame_mutex;
SemaphoreHandle_t s_state_mutex;
SemaphoreHandle_t s_command_mutex;
TaskHandle_t s_task;
uint8_t *s_frame;
uint32_t s_frame_width;
uint32_t s_frame_height;
bool s_frame_swapped;
bool s_frame_ready;
bool s_frame_dirty;
bool s_process_requested;
volatile bool s_accepting_commands;
int64_t s_last_capture_us;
int64_t s_enroll_deadline_us;
FaceCommand s_command = FaceCommand::NONE;
td_face_status_t s_status;
uint16_t s_candidate_face;
uint16_t s_greeted_face;
int64_t s_candidate_seen_us;
int64_t s_greeted_seen_us;
int64_t s_last_greeting_us;

void consider_greeting(uint16_t face_id)
{
    const int64_t now = esp_timer_get_time();
    constexpr int64_t continuity_us = 3000000;
    constexpr int64_t cooldown_us = 8000000;
    if (face_id == 0) {
        s_candidate_face = 0;
        return;
    }
    if (face_id == s_greeted_face && now - s_greeted_seen_us <= continuity_us) {
        s_greeted_seen_us = now;
        return;
    }
    // Require two consecutive successful recognitions of the same ID.
    bool confirmed = face_id == s_candidate_face &&
                     now - s_candidate_seen_us <= continuity_us;
    s_candidate_face = face_id;
    s_candidate_seen_us = now;
    if (confirmed && (s_last_greeting_us == 0 || now - s_last_greeting_us >= cooldown_us) &&
        voice_greet_async(face_id) == ESP_OK) {
        s_greeted_face = face_id;
        s_greeted_seen_us = now;
        s_last_greeting_us = now;
    }
}

void set_message(td_face_status_t *status, const char *text)
{
    snprintf(status->message, sizeof(status->message), "%s", text);
}

void update_status(const td_face_status_t &status)
{
    if (s_state_mutex == nullptr) {
        return;
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_status = status;
    xSemaphoreGive(s_state_mutex);
}

FaceCommand take_command(void)
{
    if (s_command_mutex == nullptr) {
        return FaceCommand::NONE;
    }
    xSemaphoreTake(s_command_mutex, portMAX_DELAY);
    FaceCommand command = s_command;
    s_command = FaceCommand::NONE;
    xSemaphoreGive(s_command_mutex);
    return command;
}

void queue_command(FaceCommand command)
{
    if (s_command_mutex == nullptr) {
        return;
    }
    xSemaphoreTake(s_command_mutex, portMAX_DELAY);
    s_command = command;
    xSemaphoreGive(s_command_mutex);
}

const dl::detect::result_t *largest_face(
    const std::list<dl::detect::result_t> &faces)
{
    const dl::detect::result_t *largest = nullptr;
    for (const auto &face : faces) {
        if (largest == nullptr || face.box_area() > largest->box_area()) {
            largest = &face;
        }
    }
    return largest;
}

void run_inference(const dl::image::img_t &image, bool enrollment_attempt)
{
    td_face_status_t status;
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        status = s_status;
        xSemaphoreGive(s_state_mutex);
    }
    status.ready = true;
    s_accepting_commands = true;
    status.face_found = false;
    status.recognized = false;
    status.face_id = 0;
    status.similarity = 0;

    std::list<dl::detect::result_t> &faces = s_detect->run(image);
    ++status.inference_count;
    const dl::detect::result_t *face = largest_face(faces);
    if (face == nullptr) {
        status.enrolling = enrollment_attempt;
        status.width = 0;
        status.height = 0;
        set_message(&status, enrollment_attempt ?
                    "请正对摄像头" : "未检测到人脸");
        update_status(status);
        consider_greeting(0);
        return;
    }

    status.face_found = true;
    status.x = static_cast<uint16_t>(std::max(face->box[0], 0));
    status.y = static_cast<uint16_t>(std::max(face->box[1], 0));
    status.width =
        static_cast<uint16_t>(std::max(face->box[2] - face->box[0], 0));
    status.height =
        static_cast<uint16_t>(std::max(face->box[3] - face->box[1], 0));

    std::vector<dl::recognition::result_t> matches =
        s_recognizer->recognize(image, faces);
    if (!matches.empty()) {
        status.recognized = true;
        status.face_id = matches.front().id;
        status.similarity = static_cast<uint8_t>(
            std::clamp(matches.front().similarity, 0.0f, 1.0f) * 100.0f);
        status.registered_count =
            static_cast<uint16_t>(s_recognizer->get_num_feats());
        status.enrolling = false;
        if (enrollment_attempt) {
            s_enroll_deadline_us = 0;
        }
        snprintf(status.message, sizeof(status.message),
                 enrollment_attempt ? "已登记：人员 %03u" :
                 "识别：人员 %03u  %u%%",
                 status.face_id,
                 enrollment_attempt ? status.face_id : status.similarity);
        update_status(status);
        consider_greeting(enrollment_attempt ? 0 : status.face_id);
        return;
    }

    consider_greeting(0);
    if (enrollment_attempt) {
        status.enrolling = true;
        esp_err_t err = s_recognizer->enroll(image, faces);
        status.registered_count =
            static_cast<uint16_t>(s_recognizer->get_num_feats());
        if (err == ESP_OK) {
            status.enrolling = false;
            status.face_id = status.registered_count;
            status.recognized = true;
            snprintf(status.message, sizeof(status.message),
                     "登记成功：人员 %03u", status.face_id);
            s_enroll_deadline_us = 0;
            ESP_LOGI(TAG, "face enrolled, total=%u",
                     status.registered_count);
        } else {
            set_message(&status, "人脸登记失败");
        }
    } else {
        status.enrolling = false;
        status.registered_count =
            static_cast<uint16_t>(s_recognizer->get_num_feats());
        set_message(&status, status.registered_count == 0 ?
                    "人脸库为空" : "未登记人员");
    }
    update_status(status);
}

void face_task(void *arg)
{
    (void)arg;
    td_face_status_t status = {};
    set_message(&status, "正在加载人脸模型");
    update_status(status);

    esp_vfs_spiffs_conf_t spiffs = {
        .base_path = "/storage",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_spiffs_register(&spiffs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        set_message(&status, "人脸库挂载失败");
        update_status(status);
        vTaskDelete(nullptr);
        return;
    }

    s_detect =
        new (std::nothrow) HumanFaceDetect(HumanFaceDetect::MSRMNP_S8_V1,
                                          false);
    s_recognizer = new (std::nothrow) HumanFaceRecognizer(
        DB_PATH, HumanFaceFeat::MFN_S8_V1, false);
    if (s_detect == nullptr || s_recognizer == nullptr) {
        ESP_LOGE(TAG, "face model allocation failed");
        status.ready = false;
        set_message(&status, "人脸模型内存不足");
        update_status(status);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    status.ready = true;
    status.registered_count =
        static_cast<uint16_t>(s_recognizer->get_num_feats());
    set_message(&status, status.registered_count == 0 ?
                "人脸识别就绪，人脸库为空" : "人脸识别就绪");
    update_status(status);
    ESP_LOGI(TAG, "face recognition ready, enrolled=%u",
             status.registered_count);

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        FaceCommand command = take_command();
        if (command == FaceCommand::DELETE_LAST) {
            esp_err_t delete_err = s_recognizer->delete_last_feat();
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_status.enrolling = false;
            s_status.face_found = false;
            s_status.recognized = false;
            s_status.registered_count =
                static_cast<uint16_t>(s_recognizer->get_num_feats());
            set_message(&s_status, delete_err == ESP_OK ?
                        "已删除最后一条人脸" : "没有可删除的人脸");
            xSemaphoreGive(s_state_mutex);
        } else if (command == FaceCommand::CLEAR_ALL) {
            esp_err_t clear_err = s_recognizer->clear_all_feats();
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_status.enrolling = false;
            s_status.face_found = false;
            s_status.recognized = false;
            s_status.registered_count = 0;
            set_message(&s_status, clear_err == ESP_OK ?
                        "人脸库已清空" : "清空人脸库失败");
            xSemaphoreGive(s_state_mutex);
        }

        if (command == FaceCommand::ENROLL) {
            s_enroll_deadline_us = esp_timer_get_time() + ENROLL_TIMEOUT_US;
            s_process_requested = true;
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_status.enrolling = true;
            set_message(&s_status, "请正对摄像头，8 秒内完成登记");
            xSemaphoreGive(s_state_mutex);
        }

        bool enrollment_attempt =
            s_enroll_deadline_us != 0 &&
            esp_timer_get_time() <= s_enroll_deadline_us;
        if (s_enroll_deadline_us != 0 && !enrollment_attempt) {
            s_enroll_deadline_us = 0;
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_status.enrolling = false;
            set_message(&s_status, "人脸登记已超时，请重试");
            xSemaphoreGive(s_state_mutex);
        }

        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        bool should_process = s_frame_ready &&
            (s_frame_dirty || s_process_requested || enrollment_attempt);
        if (should_process) {
            dl::image::img_t image = {
                .data = s_frame,
                .width = static_cast<uint16_t>(s_frame_width),
                .height = static_cast<uint16_t>(s_frame_height),
                .pix_type = s_frame_swapped ?
                    dl::image::DL_IMAGE_PIX_TYPE_RGB565BE :
                    dl::image::DL_IMAGE_PIX_TYPE_RGB565LE,
            };
            s_frame_dirty = false;
            s_process_requested = false;
            run_inference(image, enrollment_attempt);
        }
        xSemaphoreGive(s_frame_mutex);
    }
}

} // namespace

extern "C" esp_err_t face_service_start(void)
{
    s_frame_mutex = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();
    s_command_mutex = xSemaphoreCreateMutex();
    if (s_frame_mutex == nullptr || s_state_mutex == nullptr ||
        s_command_mutex == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    const size_t frame_size =
        static_cast<size_t>(TD_CAM_VIEW_WIDTH) * TD_CAM_VIEW_HEIGHT * 2u;
    s_frame = static_cast<uint8_t *>(
        heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_frame == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_status, 0, sizeof(s_status));
    set_message(&s_status, "人脸功能启动中");

    if (xTaskCreatePinnedToCore(face_task, "face_ai", 32768, nullptr, 3,
                                &s_task, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

extern "C" void face_service_submit_frame(const uint8_t *rgb565,
                                          uint32_t width,
                                          uint32_t height,
                                          bool swapped)
{
    if (rgb565 == nullptr || width != TD_CAM_VIEW_WIDTH ||
        height != TD_CAM_VIEW_HEIGHT || s_task == nullptr ||
        s_frame_mutex == nullptr) {
        return;
    }
    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_capture_us < CAPTURE_INTERVAL_US) {
        return;
    }
    if (xSemaphoreTake(s_frame_mutex, 0) != pdTRUE) {
        return;
    }
    memcpy(s_frame, rgb565,
           static_cast<size_t>(width) * height * 2u);
    s_frame_width = width;
    s_frame_height = height;
    s_frame_swapped = swapped;
    s_frame_ready = true;
    s_frame_dirty = true;
    s_last_capture_us = now_us;
    xSemaphoreGive(s_frame_mutex);
    xTaskNotifyGive(s_task);
}

extern "C" esp_err_t face_service_request_enroll(void)
{
    if (!s_accepting_commands || s_task == nullptr ||
        s_frame_mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    queue_command(FaceCommand::ENROLL);
    xTaskNotifyGive(s_task);
    return ESP_OK;
}

extern "C" esp_err_t face_service_delete_last(void)
{
    if (!s_accepting_commands || s_task == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    queue_command(FaceCommand::DELETE_LAST);
    xTaskNotifyGive(s_task);
    return ESP_OK;
}

extern "C" esp_err_t face_service_clear_all(void)
{
    if (!s_accepting_commands || s_task == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    queue_command(FaceCommand::CLEAR_ALL);
    xTaskNotifyGive(s_task);
    return ESP_OK;
}

extern "C" void face_service_snapshot(td_face_status_t *out)
{
    if (out == nullptr) {
        return;
    }
    if (s_state_mutex == nullptr) {
        memset(out, 0, sizeof(*out));
        set_message(out, "人脸功能未启动");
        return;
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_state_mutex);
}
