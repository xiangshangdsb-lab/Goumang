/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include "esp_err.h"
#include "esp_log.h"
#include <inttypes.h>
#include "linux/videodev2.h"
#include "esp_video_ioctl.h"
#include "app_video.h"
#include "bsp/esp-bsp.h"

static const char *TAG = "app_video";

#define MAX_BUFFER_COUNT                (6)
#define MIN_BUFFER_COUNT                (2)
#define VIDEO_DEQUEUE_TIMEOUT_US        250000

typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    uint32_t camera_pixelformat;
    uint8_t camera_mem_mode;
    app_video_frame_operation_cb_t user_camera_video_frame_operation_cb;
} app_video_t;

static app_video_t app_camera_video;

uint32_t app_video_get_pixelformat(void)
{
    return app_camera_video.camera_pixelformat;
}

static void log_v4l2_fourcc(uint32_t f, const char *prefix)
{
    char s[5] = {
        (char)(f & 0xff),
        (char)((f >> 8) & 0xff),
        (char)((f >> 16) & 0xff),
        (char)((f >> 24) & 0xff),
        0
    };
    ESP_LOGI(TAG, "%s: %s (0x%08" PRIx32 ")", prefix, s, f);
}

int app_video_open(char *dev, video_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "Open video failed");
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "failed to get capability");
        goto exit_0;
    }

    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability.version >> 16),
             (uint8_t)(capability.version >> 8),
             (uint8_t)capability.version);
    ESP_LOGI(TAG, "driver:  %s", capability.driver);
    ESP_LOGI(TAG, "card:    %s", capability.card);
    ESP_LOGI(TAG, "bus:     %s", capability.bus_info);

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        goto exit_0;
    }

    ESP_LOGI(TAG, "width=%" PRIu32 " height=%" PRIu32, default_format.fmt.pix.width, default_format.fmt.pix.height);
    log_v4l2_fourcc(default_format.fmt.pix.pixelformat, "initial pixel format");

    if (init_fmt != APP_VIDEO_FMT_DRIVER_DEFAULT &&
            default_format.fmt.pix.pixelformat != (uint32_t)init_fmt) {
        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width = default_format.fmt.pix.width,
            .fmt.pix.height = default_format.fmt.pix.height,
            .fmt.pix.pixelformat = init_fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "failed to set format");
            goto exit_0;
        }
    }

#if defined(BSP_CAMERA_VFLIP) && defined(BSP_CAMERA_HFLIP)
    bool vflip = BSP_CAMERA_VFLIP;
    bool hflip = BSP_CAMERA_HFLIP;
    struct v4l2_ext_control control[1];
    struct v4l2_ext_controls ctrl = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count      = 1,
        .controls   = control,
    };

    /* Set VFLIP */
    control[0].id = V4L2_CID_VFLIP;
    control[0].value = vflip;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrl) != 0) {
        ESP_LOGE(TAG, "failed to set vflip");
    }

    /* Set HFLIP */
    control[0].id = V4L2_CID_HFLIP;
    control[0].value = hflip;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrl) != 0) {
        ESP_LOGE(TAG, "failed to set hflip");
    }
#endif

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "failed to get final format");
        goto exit_0;
    }

    app_camera_video.camera_buf_hes = default_format.fmt.pix.width;
    app_camera_video.camera_buf_ves = default_format.fmt.pix.height;
    app_camera_video.camera_pixelformat = default_format.fmt.pix.pixelformat;
    log_v4l2_fourcc(app_camera_video.camera_pixelformat, "capture pixel format");

    struct timeval timeout = { .tv_usec = VIDEO_DEQUEUE_TIMEOUT_US };
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGE(TAG, "failed to set capture timeout");
        goto exit_0;
    }

    return fd;
exit_0:
    close(fd);
    return -1;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer num is too large");
        return ESP_FAIL;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "At least two buffers are required");
        return ESP_FAIL;
    }

    struct v4l2_requestbuffers req;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    memset(&req, 0, sizeof(req));
    req.count = fb_num;
    req.type = type;

    app_camera_video.camera_mem_mode = req.memory = fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "req bufs failed");
        goto errout_req_bufs;
    }
    for (int i = 0; i < fb_num; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = req.memory;
        buf.index = i;

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "query buf failed");
            goto errout_req_bufs;
        }

        if (req.memory == V4L2_MEMORY_MMAP) {
            app_camera_video.camera_buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
            if (app_camera_video.camera_buffer[i] == MAP_FAILED) {
                app_camera_video.camera_buffer[i] = NULL;
                ESP_LOGE(TAG, "mmap failed");
                goto errout_req_bufs;
            }
        } else {
            if (!fb[i]) {
                ESP_LOGE(TAG, "frame buffer is NULL");
                goto errout_req_bufs;
            }
            buf.m.userptr = (unsigned long)fb[i];
            app_camera_video.camera_buffer[i] = (uint8_t *)fb[i];
        }

        app_camera_video.camera_buf_size = buf.length;

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "queue frame buffer failed");
            goto errout_req_bufs;
        }
    }

    return ESP_OK;

errout_req_bufs:
    return ESP_FAIL;
}

esp_err_t app_video_get_bufs(int fb_num, void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer num is too large");
        return ESP_FAIL;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "At least two buffers are required");
        return ESP_FAIL;
    }

    for (int i = 0; i < fb_num; i++) {
        if (app_camera_video.camera_buffer[i] != NULL) {
            fb[i] = app_camera_video.camera_buffer[i];
        } else {
            ESP_LOGE(TAG, "frame buffer is NULL");
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t app_video_capture_frame(int video_fd)
{
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = app_camera_video.camera_mem_mode,
    };
    if (ioctl(video_fd, VIDIOC_DQBUF, &buf) != 0) {
        return ESP_ERR_TIMEOUT;
    }
    if (buf.index >= MAX_BUFFER_COUNT ||
        app_camera_video.camera_buffer[buf.index] == NULL) {
        ESP_LOGE(TAG, "invalid capture buffer index: %" PRIu32, buf.index);
        return ESP_FAIL;
    }

    size_t frame_size = (size_t)app_camera_video.camera_buf_hes *
                        app_camera_video.camera_buf_ves * 2u;
    bool valid = !(buf.flags & V4L2_BUF_FLAG_ERROR) &&
                 buf.bytesused >= frame_size &&
                 buf.bytesused <= app_camera_video.camera_buf_size;
    if (valid && app_camera_video.user_camera_video_frame_operation_cb != NULL) {
        app_camera_video.user_camera_video_frame_operation_cb(
            app_camera_video.camera_buffer[buf.index],
            buf.index,
            app_camera_video.camera_buf_hes,
            app_camera_video.camera_buf_ves,
            buf.bytesused);
    }
    if (buf.memory == V4L2_MEMORY_USERPTR) {
        buf.m.userptr = (unsigned long)app_camera_video.camera_buffer[buf.index];
        buf.length = app_camera_video.camera_buf_size;
    }
    if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGE(TAG, "failed to requeue video frame");
        return ESP_FAIL;
    }
    return valid ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t app_video_stream_start(int video_fd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "failed to start video stream");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t app_video_stream_stop(int video_fd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(video_fd, VIDIOC_STREAMOFF, &type) != 0) {
        ESP_LOGW(TAG, "failed to stop video stream");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb)
{
    app_camera_video.user_camera_video_frame_operation_cb = operation_cb;
    return ESP_OK;
}

esp_err_t app_video_close(int video_fd)
{
    for (int i = 0; i < MAX_BUFFER_COUNT; ++i) {
        app_camera_video.camera_buffer[i] = NULL;
    }
    return close(video_fd) == 0 ? ESP_OK : ESP_FAIL;
}
