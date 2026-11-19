#include "object_service.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <list>
#include <new>
#include <vector>

#include "app/app_config.h"
#include "coco_detect.hpp"
#include "dl_image_define.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "objects";
constexpr uint32_t CAPTURE_INTERVAL_US = 700000;
constexpr float MIN_SCORE = 0.60f;

const char *coco_name_zh(int category)
{
    static const char *names[] = {
        "人", "自行车", "汽车", "摩托车", "飞机", "公交车", "火车", "卡车",
        "船", "红绿灯", "消防栓", "停车标志", "停车计时器", "长椅", "鸟", "猫",
        "狗", "马", "羊", "牛", "大象", "熊", "斑马", "长颈鹿",
        "背包", "雨伞", "手提包", "领带", "行李箱", "飞盘", "滑雪板",
        "单板滑雪板", "球", "风筝", "棒球棒", "棒球手套", "滑板", "冲浪板",
        "网球拍", "瓶子", "酒杯", "杯子", "叉子", "刀", "勺子", "碗",
        "香蕉", "苹果", "三明治", "橙子", "西兰花", "胡萝卜", "热狗",
        "披萨", "甜甜圈", "蛋糕", "椅子", "沙发", "盆栽植物", "床",
        "餐桌", "马桶", "电视", "笔记本电脑", "鼠标", "遥控器", "键盘",
        "手机", "微波炉", "烤箱", "烤面包机", "水槽", "冰箱", "书",
        "时钟", "花瓶", "剪刀", "泰迪熊", "吹风机", "牙刷",
    };
    constexpr int count = sizeof(names) / sizeof(names[0]);
    return category >= 0 && category < count ? names[category] : "未知物体";
}

COCODetect *s_detect;
SemaphoreHandle_t s_frame_mutex;
SemaphoreHandle_t s_state_mutex;
TaskHandle_t s_task;
uint8_t *s_frame;
uint32_t s_frame_width;
uint32_t s_frame_height;
bool s_frame_swapped;
bool s_frame_ready;
bool s_frame_dirty;
int64_t s_last_capture_us;
td_object_status_t s_status;

void run_inference(const dl::image::img_t &image)
{
    const int64_t start_us = esp_timer_get_time();
    td_object_status_t status = {};
    status.ready = true;
    std::list<dl::detect::result_t> &results = s_detect->run(image);
    std::vector<const dl::detect::result_t *> selected;
    selected.reserve(results.size());
    for (const auto &result : results) {
        if (result.score >= MIN_SCORE) {
            selected.push_back(&result);
        }
    }
    std::sort(selected.begin(), selected.end(),
              [](const dl::detect::result_t *a,
                 const dl::detect::result_t *b) {
                  return a->score > b->score;
              });
    status.count = static_cast<uint8_t>(
        std::min(selected.size(), static_cast<size_t>(TD_MAX_OBJECTS)));
    for (uint8_t i = 0; i < status.count; ++i) {
        const dl::detect::result_t *result = selected[i];
        td_object_t &item = status.items[i];
        item.category = static_cast<uint8_t>(result->category);
        item.confidence = static_cast<uint8_t>(
            std::clamp(result->score, 0.0f, 1.0f) * 100.0f);
        item.x = static_cast<uint16_t>(std::max(result->box[0], 0));
        item.y = static_cast<uint16_t>(std::max(result->box[1], 0));
        item.width = static_cast<uint16_t>(
            std::max(result->box[2] - result->box[0], 0));
        item.height = static_cast<uint16_t>(
            std::max(result->box[3] - result->box[1], 0));
        snprintf(item.name, sizeof(item.name), "%s",
                 coco_name_zh(result->category));
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    status.inference_count = s_status.inference_count + 1;
    s_status = status;
    xSemaphoreGive(s_state_mutex);

    if (status.inference_count == 1u ||
        status.inference_count % 30u == 0u) {
        if (status.count > 0u) {
            ESP_LOGI(TAG, "inference #%lu: %u objects, top=%s %u%%, %lld ms",
                     (unsigned long)status.inference_count, status.count,
                     status.items[0].name, status.items[0].confidence,
                     (esp_timer_get_time() - start_us) / 1000);
        } else {
            ESP_LOGI(TAG, "inference #%lu: no object, %lld ms",
                     (unsigned long)status.inference_count,
                     (esp_timer_get_time() - start_us) / 1000);
        }
    }
}

void object_task(void *arg)
{
    (void)arg;
    /* Let the face detector allocate first; ESP-DL memory planning is global. */
    vTaskDelay(pdMS_TO_TICKS(10000));
    s_detect =
        new (std::nothrow) COCODetect(COCODetect::YOLO11N_320_S8_V1, false);
    if (s_detect == nullptr) {
        ESP_LOGE(TAG, "COCO model allocation failed");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_status.ready = true;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "COCO YOLO11n-320 object detector ready");

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }
        if (s_frame_ready && s_frame_dirty) {
            dl::image::img_t image = {
                .data = s_frame,
                .width = static_cast<uint16_t>(s_frame_width),
                .height = static_cast<uint16_t>(s_frame_height),
                .pix_type = s_frame_swapped ?
                    dl::image::DL_IMAGE_PIX_TYPE_RGB565BE :
                    dl::image::DL_IMAGE_PIX_TYPE_RGB565LE,
            };
            s_frame_dirty = false;
            run_inference(image);
        }
        xSemaphoreGive(s_frame_mutex);
    }
}

} // namespace

extern "C" esp_err_t object_service_start(void)
{
    s_frame_mutex = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_frame_mutex == nullptr || s_state_mutex == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    const size_t frame_size =
        static_cast<size_t>(TD_CAM_VIEW_WIDTH) * TD_CAM_VIEW_HEIGHT * 2u;
    s_frame = static_cast<uint8_t *>(
        heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_frame == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(object_task, "object_ai", 32768, nullptr, 2,
                                &s_task, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

extern "C" void object_service_submit_frame(const uint8_t *rgb565,
                                            uint32_t width,
                                            uint32_t height,
                                            bool swapped)
{
    if (rgb565 == nullptr || width != TD_CAM_VIEW_WIDTH ||
        height != TD_CAM_VIEW_HEIGHT || s_task == nullptr) {
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

extern "C" void object_service_snapshot(td_object_status_t *out)
{
    if (out == nullptr) {
        return;
    }
    if (s_state_mutex == nullptr) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    *out = s_status;
    xSemaphoreGive(s_state_mutex);
}
