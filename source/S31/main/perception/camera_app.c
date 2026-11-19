#include "camera_app.h"
#include "esp_jpeg_enc.h"

#include <stdatomic.h>
#include <string.h>

#include "app/app_config.h"
#include "app/app_state.h"
#include "app_video.h"
#include "bsp/esp-bsp.h"
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "esp_video_init.h"
#include "face_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "object_service.h"
#include "rgb565_format.h"
#include "interface/panel/ui.h"
#include "vision.h"

#define CAMERA_BUFFER_COUNT       2
#define WEB_SNAPSHOT_WIDTH        480
#define WEB_SNAPSHOT_HEIGHT       360
#define WEB_SNAPSHOT_INTERVAL_US  500000
#define LCD_PREVIEW_INTERVAL_US   100000
#define CAMERA_FRAME_STALE_MS     1000u
#define CAMERA_STREAM_STALL_MS     2000u
#define CAMERA_RETRY_MS            500u
#define CAMERA_RETRY_MAX_MS        4000u
#define ALIGN_UP(num, align)      (((num) + ((align) - 1)) & ~((align) - 1))

#if BSP_CAMERA_ROTATION == 0
#define TD_CAMERA_PPA_ROTATION PPA_SRM_ROTATION_ANGLE_0
#elif BSP_CAMERA_ROTATION == 90
#define TD_CAMERA_PPA_ROTATION PPA_SRM_ROTATION_ANGLE_90
#elif BSP_CAMERA_ROTATION == 180
#define TD_CAMERA_PPA_ROTATION PPA_SRM_ROTATION_ANGLE_180
#elif BSP_CAMERA_ROTATION == 270
#define TD_CAMERA_PPA_ROTATION PPA_SRM_ROTATION_ANGLE_270
#else
#error "BSP_CAMERA_ROTATION must be 0, 90, 180, or 270"
#endif

static const char *TAG = "camera";

static ppa_client_handle_t s_ppa;
static uint8_t *s_camera_buffer;
static uint8_t *s_display_buffer;
static size_t s_camera_buffer_size;
static lv_obj_t *s_canvas;
static ppa_srm_color_mode_t s_input_color_mode = PPA_SRM_COLOR_MODE_RGB565;
static bool s_input_byte_swap;
static uint8_t *s_web_snapshot;
static SemaphoreHandle_t s_snapshot_mutex;
static int64_t s_last_snapshot_us;
static int64_t s_last_lcd_preview_us;
static _Atomic uint32_t s_last_frame_ms;

static bool configure_pixel_format(uint32_t pixelformat)
{
    if (pixelformat == V4L2_PIX_FMT_RGB565X) {
        s_input_color_mode = PPA_SRM_COLOR_MODE_RGB565;
        s_input_byte_swap = true;
        return true;
    }
    if (pixelformat == V4L2_PIX_FMT_UYVY) {
        s_input_color_mode = PPA_SRM_COLOR_MODE_YUV422_UYVY;
        s_input_byte_swap = false;
        return true;
    }
    if (pixelformat == V4L2_PIX_FMT_YUYV) {
        s_input_color_mode = PPA_SRM_COLOR_MODE_YUV422_YUYV;
        s_input_byte_swap = false;
        return true;
    }
    s_input_color_mode = PPA_SRM_COLOR_MODE_RGB565;
    s_input_byte_swap = false;
    return pixelformat == V4L2_PIX_FMT_RGB565;
}

static void copy_web_snapshot(const uint8_t *frame)
{
    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_snapshot_us < WEB_SNAPSHOT_INTERVAL_US) {
        return;
    }
    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(s_web_snapshot, frame,
               (size_t)TD_CAM_VIEW_WIDTH * TD_CAM_VIEW_HEIGHT * 2u);
        s_last_snapshot_us = now_us;
        xSemaphoreGive(s_snapshot_mutex);
    }
}

static esp_err_t ppa_copy_scale(uint8_t *input,
                                uint32_t input_width,
                                uint32_t input_height,
                                uint8_t *output,
                                uint32_t output_width,
                                uint32_t output_height)
{
    ppa_srm_oper_config_t config = {
        .in.buffer = input,
        .in.pic_w = input_width,
        .in.pic_h = input_height,
        .in.block_w = input_width,
        .in.block_h = input_height,
        .in.block_offset_x = 0,
        .in.block_offset_y = 0,
        .in.srm_cm = s_input_color_mode,
        .in.yuv_range = PPA_COLOR_RANGE_FULL,
        .in.yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601,
        .out.buffer = output,
        .out.buffer_size = s_camera_buffer_size,
        .out.pic_w = output_width,
        .out.pic_h = output_height,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle = TD_CAMERA_PPA_ROTATION,
        .scale_x = (float)output_width / input_width,
        .scale_y = (float)output_height / input_height,
        .rgb_swap = 0,
        .byte_swap = s_input_byte_swap,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    return ppa_do_scale_rotate_mirror(s_ppa, &config);
}

static void frame_callback(uint8_t *camera_buf,
                           uint8_t camera_buf_index,
                           uint32_t camera_buf_hes,
                           uint32_t camera_buf_ves,
                           size_t camera_buf_len)
{
    (void)camera_buf_len;
    if (camera_buf_index >= CAMERA_BUFFER_COUNT) {
        return;
    }
    /* Only the capture task uses this buffer. Consumers copy the frame. */
    uint8_t *out_buf = s_camera_buffer;
    esp_err_t err = ppa_copy_scale(camera_buf, camera_buf_hes,
                                  camera_buf_ves, out_buf,
                                  TD_CAM_VIEW_WIDTH, TD_CAM_VIEW_HEIGHT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPA conversion failed: %s", esp_err_to_name(err));
        return;
    }

    bool first_frame = atomic_exchange(&s_last_frame_ms, app_now_ms()) == 0;
    if (first_frame) {
        app_state_lock();
        app_state_mutable()->camera_ready = true;
        app_state_mutable()->camera_streaming = true;
        app_state_unlock();
        ESP_LOGI(TAG, "first camera frame received");
    }

    vision_process_frame(out_buf, TD_CAM_VIEW_WIDTH, TD_CAM_VIEW_HEIGHT);
    face_service_submit_frame(out_buf, TD_CAM_VIEW_WIDTH, TD_CAM_VIEW_HEIGHT,
                              false);
    object_service_submit_frame(out_buf, TD_CAM_VIEW_WIDTH,
                                TD_CAM_VIEW_HEIGHT, false);
    copy_web_snapshot(out_buf);

    int64_t now_us = esp_timer_get_time();
    if (now_us - s_last_lcd_preview_us >= LCD_PREVIEW_INTERVAL_US &&
        bsp_display_lock(20)) {
        s_last_lcd_preview_us = now_us;
        /*
         * PPA working buffers are overwritten at the sensor frame rate.
         * LVGL must own a separate buffer; otherwise the active canvas can
         * be overwritten while the display task is reading it.
         */
        memcpy(s_display_buffer, out_buf,
               (size_t)TD_CAM_VIEW_WIDTH * TD_CAM_VIEW_HEIGHT * 2u);
        lv_obj_invalidate(s_canvas);
        bsp_display_unlock();
    }
}

static void write_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void write_le32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static bool camera_snapshot_is_fresh(void)
{
    uint32_t last_frame_ms = atomic_load(&s_last_frame_ms);
    return last_frame_ms != 0u &&
           (uint32_t)(app_now_ms() - last_frame_ms) <= CAMERA_FRAME_STALE_MS;
}

static bool web_snapshot_fresh_locked(int64_t min_capture_us)
{
    int64_t now_us = app_now_us();
    return camera_snapshot_is_fresh() && s_last_snapshot_us > 0 &&
           s_last_snapshot_us >= min_capture_us &&
           now_us - s_last_snapshot_us <= (int64_t)CAMERA_FRAME_STALE_MS * 1000;
}

bool camera_app_has_fresh_frame(void)
{
    return camera_snapshot_is_fresh();
}

esp_err_t camera_app_get_bmp(uint8_t **data, size_t *length)
{
    if (data == NULL || length == NULL || s_web_snapshot == NULL ||
        !camera_snapshot_is_fresh()) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t row_size = WEB_SNAPSHOT_WIDTH * 3u;
    const size_t pixel_size = row_size * WEB_SNAPSHOT_HEIGHT;
    const size_t file_size = 54u + pixel_size;
    uint8_t *bmp = heap_caps_malloc(file_size,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (bmp == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(bmp, 0, 54);
    bmp[0] = 'B';
    bmp[1] = 'M';
    write_le32(&bmp[2], (uint32_t)file_size);
    write_le32(&bmp[10], 54);
    write_le32(&bmp[14], 40);
    write_le32(&bmp[18], WEB_SNAPSHOT_WIDTH);
    write_le32(&bmp[22], WEB_SNAPSHOT_HEIGHT);
    write_le16(&bmp[26], 1);
    write_le16(&bmp[28], 24);
    write_le32(&bmp[34], (uint32_t)pixel_size);

    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        heap_caps_free(bmp);
        return ESP_ERR_TIMEOUT;
    }

    if (!web_snapshot_fresh_locked(0)) {
        xSemaphoreGive(s_snapshot_mutex);
        heap_caps_free(bmp);
        return ESP_ERR_INVALID_STATE;
    }
    for (uint32_t y = 0; y < WEB_SNAPSHOT_HEIGHT; ++y) {
        uint32_t src_y = y * TD_CAM_VIEW_HEIGHT / WEB_SNAPSHOT_HEIGHT;
        uint8_t *dst = bmp + 54u +
            (size_t)(WEB_SNAPSHOT_HEIGHT - 1u - y) * row_size;
        for (uint32_t x = 0; x < WEB_SNAPSHOT_WIDTH; ++x) {
            uint32_t src_x = x * TD_CAM_VIEW_WIDTH / WEB_SNAPSHOT_WIDTH;
            const uint8_t *src =
                s_web_snapshot +
                ((size_t)src_y * TD_CAM_VIEW_WIDTH + src_x) * 2u;
            uint16_t pixel = td_rgb565_read_le(src);
            uint8_t r5 = (uint8_t)((pixel >> 11) & 0x1F);
            uint8_t g6 = (uint8_t)((pixel >> 5) & 0x3F);
            uint8_t b5 = (uint8_t)(pixel & 0x1F);
            dst[x * 3u + 0u] = (uint8_t)((b5 * 255u + 15u) / 31u);
            dst[x * 3u + 1u] = (uint8_t)((g6 * 255u + 31u) / 63u);
            dst[x * 3u + 2u] = (uint8_t)((r5 * 255u + 15u) / 31u);
        }
    }
    xSemaphoreGive(s_snapshot_mutex);

    *data = bmp;
    *length = file_size;
    return ESP_OK;
}

esp_err_t camera_app_get_jpeg(int64_t min_capture_us, uint8_t **data, size_t *length)
{
    /* 4:2:0 encodes 16-row blocks. Trim four rows from each edge. */
    const int height = TD_CAM_VIEW_HEIGHT / 16 * 16;
    const size_t row_bytes = TD_CAM_VIEW_WIDTH * 2u;
    const size_t pixel_bytes = row_bytes * height;
    const int jpeg_capacity = 128 * 1024;
    uint8_t *pixels;
    uint8_t *jpeg;
    jpeg_enc_config_t cfg = DEFAULT_JPEG_ENC_CONFIG();
    jpeg_enc_handle_t encoder = NULL;
    int jpeg_length = 0;

    if (data == NULL || length == NULL || !camera_snapshot_is_fresh() ||
        s_web_snapshot == NULL || s_snapshot_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    pixels = heap_caps_aligned_alloc(16, pixel_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (pixels == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        heap_caps_free(pixels);
        return ESP_ERR_TIMEOUT;
    }
    bool fresh = web_snapshot_fresh_locked(min_capture_us);
    if (fresh) {
        size_t crop = (TD_CAM_VIEW_HEIGHT - height) / 2u * row_bytes;
        memcpy(pixels, s_web_snapshot + crop, pixel_bytes);
    }
    xSemaphoreGive(s_snapshot_mutex);
    if (!fresh) {
        heap_caps_free(pixels);
        return ESP_ERR_NOT_FOUND;
    }

    jpeg = heap_caps_malloc(jpeg_capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (jpeg == NULL) {
        heap_caps_free(pixels);
        return ESP_ERR_NO_MEM;
    }
    cfg.width = TD_CAM_VIEW_WIDTH;
    cfg.height = height;
    cfg.src_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    cfg.quality = 78;
    jpeg_error_t result = jpeg_enc_open(&cfg, &encoder);
    if (result == JPEG_ERR_OK) {
        result = jpeg_enc_process(encoder, pixels, (int)pixel_bytes, jpeg,
                                  jpeg_capacity, &jpeg_length);
        jpeg_enc_close(encoder);
    }
    heap_caps_free(pixels);
    if (result != JPEG_ERR_OK || jpeg_length <= 0) {
        heap_caps_free(jpeg);
        return ESP_FAIL;
    }
    *data = jpeg;
    *length = (size_t)jpeg_length;
    return ESP_OK;
}

void camera_app_free_bmp(uint8_t *data)
{
    heap_caps_free(data);
}

static void camera_release_resources(void)
{
    if (s_canvas != NULL) {
        ui_detach_camera();
        bsp_display_lock(0);
        lv_obj_delete(s_canvas);
        s_canvas = NULL;
        bsp_display_unlock();
    }
    heap_caps_free(s_camera_buffer);
    s_camera_buffer = NULL;
    heap_caps_free(s_display_buffer);
    s_display_buffer = NULL;
    heap_caps_free(s_web_snapshot);
    s_web_snapshot = NULL;
    if (s_snapshot_mutex != NULL) {
        vSemaphoreDelete(s_snapshot_mutex);
        s_snapshot_mutex = NULL;
    }
    if (s_ppa != NULL) {
        (void)ppa_unregister_client(s_ppa);
        s_ppa = NULL;
    }
}

static void camera_mark_offline(void)
{
    atomic_store(&s_last_frame_ms, 0u);
    app_state_lock();
    app_state_mutable()->camera_ready = false;
    app_state_mutable()->camera_streaming = false;
    app_state_unlock();

    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_last_snapshot_us = 0;
    xSemaphoreGive(s_snapshot_mutex);
}

static void camera_task(void *arg)
{
    (void)arg;
    uint32_t retry_ms = CAMERA_RETRY_MS;

    while (true) {
        int fd = -1;
        bool started = false;
        esp_err_t err = bsp_camera_start(NULL);
        if (err == ESP_OK) {
            fd = app_video_open(BSP_CAMERA_DEVICE, APP_VIDEO_FMT_DRIVER_DEFAULT);
            err = fd >= 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
        }
        if (err == ESP_OK && !configure_pixel_format(app_video_get_pixelformat())) {
            err = ESP_ERR_NOT_SUPPORTED;
        }
        if (err == ESP_OK) {
            err = app_video_set_bufs(fd, CAMERA_BUFFER_COUNT, NULL);
        }
        if (err == ESP_OK) {
            err = app_video_stream_start(fd);
            started = err == ESP_OK;
            if (!started) {
                ESP_LOGW(TAG, "camera DMA PSRAM free=%u, largest=%u",
                         (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA),
                         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
            }
        }
        if (started) {
            uint32_t started_ms = app_now_ms();
            while (true) {
                err = app_video_capture_frame(fd);
                uint32_t last_frame = atomic_load(&s_last_frame_ms);
                uint32_t reference = last_frame != 0u ? last_frame : started_ms;
                if (last_frame != 0u) {
                    retry_ms = CAMERA_RETRY_MS;
                }
                if (err == ESP_FAIL) {
                    break;
                }
                if ((uint32_t)(app_now_ms() - reference) >= CAMERA_STREAM_STALL_MS) {
                    ESP_LOGW(TAG, "%s", last_frame != 0u ? "camera stream stalled" :
                             "camera first frame timed out");
                    err = ESP_ERR_TIMEOUT;
                    break;
                }
                if (err != ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }

        camera_mark_offline();
        if (started) {
            (void)app_video_stream_stop(fd);
        }
        if (fd >= 0) {
            (void)app_video_close(fd);
        }
        esp_err_t deinit = esp_video_deinit_with_flags(ESP_VIDEO_INIT_FLAGS_DVP);
        if (deinit != ESP_OK) {
            ESP_LOGW(TAG, "camera DVP cleanup: %s", esp_err_to_name(deinit));
        }
        ESP_LOGW(TAG, "camera recovery in %lu ms: %s", (unsigned long)retry_ms,
                 esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(retry_ms));
        if (retry_ms < CAMERA_RETRY_MAX_MS) {
            retry_ms *= 2;
        }
    }
}

esp_err_t camera_app_start(void)
{
    size_t cache_line_size;
    esp_err_t err;
    ppa_client_config_t ppa_config = {
        .oper_type = PPA_OPERATION_SRM,
    };
    err = ppa_register_client(&ppa_config, &s_ppa);
    if (err != ESP_OK) {
        goto cleanup;
    }

    err = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &cache_line_size);
    if (err != ESP_OK) {
        goto cleanup;
    }
    s_camera_buffer_size =
        ALIGN_UP(TD_CAM_VIEW_WIDTH * TD_CAM_VIEW_HEIGHT * 2u,
                 cache_line_size);
    s_camera_buffer = heap_caps_aligned_calloc(
        cache_line_size, 1, s_camera_buffer_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_camera_buffer == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    s_display_buffer = heap_caps_aligned_calloc(
        cache_line_size, 1, s_camera_buffer_size,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_display_buffer == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    s_web_snapshot = heap_caps_malloc(
        (size_t)TD_CAM_VIEW_WIDTH * TD_CAM_VIEW_HEIGHT * 2u,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_web_snapshot == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    memset(s_web_snapshot, 0,
           (size_t)TD_CAM_VIEW_WIDTH * TD_CAM_VIEW_HEIGHT * 2u);
    s_snapshot_mutex = xSemaphoreCreateMutex();
    if (s_snapshot_mutex == NULL) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    bsp_display_lock(0);
    s_canvas = lv_canvas_create(lv_screen_active());
    if (s_canvas == NULL) {
        bsp_display_unlock();
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    lv_canvas_set_buffer(s_canvas, s_display_buffer,
                         TD_CAM_VIEW_WIDTH, TD_CAM_VIEW_HEIGHT,
                         LV_COLOR_FORMAT_RGB565);
    bsp_display_unlock();
    ui_attach_camera(s_canvas);

    (void)app_video_register_frame_operation_cb(frame_callback);
    if (xTaskCreatePinnedToCore(camera_task, "camera", 6144, NULL, 6, NULL, 1) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    return ESP_OK;

cleanup:
    (void)app_video_register_frame_operation_cb(NULL);
    camera_release_resources();
    return err;
}
