/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "bsp/esp32_s31_korvo_1.h"
#include "bsp_adc_calibration.h"

static const char *TAG = "S31-ADC";
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;

esp_err_t bsp_adc_initialize(void)
{
    /* ADC was initialized before */
    if (bsp_adc_handle != NULL) {
        return ESP_OK;
    }

    /* Initialize ADC */
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = BSP_ADC_UNIT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(
        adc_oneshot_new_unit(&init_config1, &bsp_adc_handle));

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_oneshot_config_channel(
        bsp_adc_handle, ADC_CHANNEL_0, &channel_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "configure ADC1 channel 0 failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = bsp_s31_adc_calibration_init(
        bsp_adc_handle, ADC_UNIT_1, ADC_CHANNEL_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "S31 ADC calibration failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    int raw_sum = 0;
    int raw = 0;
    const int sample_count = 8;
    for (int i = 0; i < sample_count; i++) {
        err = adc_oneshot_read(bsp_adc_handle, ADC_CHANNEL_0, &raw);
        if (err != ESP_OK) {
            break;
        }
        raw_sum += raw;
    }
    if (err == ESP_OK) {
        raw = raw_sum / sample_count;
        int voltage_mv = 0;
        err = bsp_s31_adc_calibration_raw_to_mv(
            ADC_UNIT_1, raw, &voltage_mv);
        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                     "calibrated idle sample: raw=%d voltage=%d mV",
                     raw, voltage_mv);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "calibrated ADC verification sample failed: %s",
                 esp_err_to_name(err));
    }

    return ESP_OK;
}

adc_oneshot_unit_handle_t bsp_adc_get_handle(void)
{
    return bsp_adc_handle;
}
