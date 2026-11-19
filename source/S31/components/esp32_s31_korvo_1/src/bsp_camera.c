/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

#include "bsp/esp32_s31_korvo_1.h"

#define BSP_CAMERA_SCCB_FREQ_HZ (10 * 1000)

esp_err_t bsp_camera_start(const bsp_camera_cfg_t *cfg)
{
    (void)cfg;

    /* Initialize the shared board I2C/SCCB bus. */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    const esp_video_init_dvp_config_t base_dvp_config = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = bsp_i2c_get_handle(),
            .freq = BSP_CAMERA_SCCB_FREQ_HZ,
        },
        .reset_pin = BSP_CAMERA_RST,
        .pwdn_pin  = -1,
        .dvp_pin = {
            .data_width = 8,
            .data_io = {
                BSP_CAMERA_D0, BSP_CAMERA_D1, BSP_CAMERA_D2, BSP_CAMERA_D3,
                BSP_CAMERA_D4, BSP_CAMERA_D5, BSP_CAMERA_D6, BSP_CAMERA_D7,
            },
            .vsync_io = BSP_CAMERA_VSYNC,
            .de_io = BSP_CAMERA_HSYNC,
            .pclk_io = BSP_CAMERA_PCLK,
            .xclk_io = BSP_CAMERA_GPIO_XCLK,
        },
        .xclk_freq = BSP_CAMERA_XCLK_CLOCK_MHZ * 1000000,
    };

    esp_video_init_config_t cam_config = {
        .dvp      = &base_dvp_config,
    };

    /*
     * Let the S31 DVP controller generate the camera XCLK. This follows the
     * official S31 factory-demo sequence. Starting a separate LEDC XCLK here
     * can probe OV3660 SCCB before the clock is stable and has been observed
     * to return NAK at address 0x3c.
     */
    return esp_video_init_with_flags(&cam_config, ESP_VIDEO_INIT_FLAGS_DVP);
}
