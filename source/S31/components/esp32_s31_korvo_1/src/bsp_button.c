/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp/esp32_s31_korvo_1.h"
#include "bsp_adc_calibration.h"
#include "esp_check.h"
#include "esp_log.h"
#include "iot_button.h"

static const char *TAG = "ESP32-S31-Korvo-1";

/* Voltage-ladder centres from the ESP32-S31-Korvo-1 schematic. */
static const uint16_t s_button_voltage_center_mv[BSP_BUTTON_NUM] = {
    [BSP_BUTTON_VOLUP] = 380,
    [BSP_BUTTON_VOLDOWN] = 820,
    [BSP_BUTTON_MODE] = 1340,
    [BSP_BUTTON_SET] = 1870,
};
static const uint16_t s_button_idle_voltage_mv = 2000;

#define BSP_BUTTON_ADC_SAMPLE_COUNT 4

typedef struct {
    button_driver_t base;
    uint8_t index;
    uint16_t min_mv;
    uint16_t max_mv;
} bsp_adc_button_t;

static bsp_adc_button_t s_button_driver[BSP_BUTTON_NUM];

static uint16_t bsp_button_mv_midpoint(uint16_t a, uint16_t b)
{
    return (uint16_t)(((uint32_t)a + b) / 2u);
}

static esp_err_t bsp_button_adc_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_adc_initialize(), TAG,
                        "initialize calibrated ADC failed");
    return ESP_OK;
}

static uint8_t bsp_button_get_key_level(button_driver_t *button_driver)
{
    bsp_adc_button_t *button = (bsp_adc_button_t *)button_driver;
    adc_oneshot_unit_handle_t handle = bsp_adc_get_handle();
    uint32_t raw_sum = 0;
    int raw = 0;

    for (int i = 0; i < BSP_BUTTON_ADC_SAMPLE_COUNT; i++) {
        if (adc_oneshot_read(handle, ADC_CHANNEL_0, &raw) != ESP_OK) {
            return BUTTON_INACTIVE;
        }
        raw_sum += (uint32_t)raw;
    }
    raw = (int)(raw_sum / BSP_BUTTON_ADC_SAMPLE_COUNT);

    int voltage_mv = 0;
    if (bsp_s31_adc_calibration_raw_to_mv(
            ADC_UNIT_1, raw, &voltage_mv) != ESP_OK) {
        return BUTTON_INACTIVE;
    }
    return voltage_mv >= button->min_mv &&
           voltage_mv <= button->max_mv ?
           BUTTON_ACTIVE : BUTTON_INACTIVE;
}

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt,
                                int btn_array_size)
{
    ESP_RETURN_ON_FALSE(btn_array != NULL &&
                        btn_array_size >= BSP_BUTTON_NUM,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid button output array");
    ESP_RETURN_ON_ERROR(bsp_button_adc_init(), TAG,
                        "button ADC init failed");

    const button_config_t button_config = {0};
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        s_button_driver[i].base.get_key_level =
            bsp_button_get_key_level;
        s_button_driver[i].index = (uint8_t)i;

        if (i == 0) {
            uint16_t upper = bsp_button_mv_midpoint(
                s_button_voltage_center_mv[i],
                s_button_voltage_center_mv[i + 1]);
            uint16_t margin =
                upper - s_button_voltage_center_mv[i];
            s_button_driver[i].min_mv =
                s_button_voltage_center_mv[i] > margin ?
                s_button_voltage_center_mv[i] - margin : 0;
            s_button_driver[i].max_mv = upper;
        } else if (i == BSP_BUTTON_NUM - 1) {
            uint16_t lower = bsp_button_mv_midpoint(
                s_button_voltage_center_mv[i - 1],
                s_button_voltage_center_mv[i]);
            s_button_driver[i].min_mv = lower;
            s_button_driver[i].max_mv = bsp_button_mv_midpoint(
                s_button_voltage_center_mv[i],
                s_button_idle_voltage_mv);
        } else {
            s_button_driver[i].min_mv = bsp_button_mv_midpoint(
                s_button_voltage_center_mv[i - 1],
                s_button_voltage_center_mv[i]);
            s_button_driver[i].max_mv = bsp_button_mv_midpoint(
                s_button_voltage_center_mv[i],
                s_button_voltage_center_mv[i + 1]);
        }

        ESP_LOGI(TAG,
                 "Button %d threshold: center=%u mV, range=%u-%u mV",
                 i, s_button_voltage_center_mv[i],
                 s_button_driver[i].min_mv,
                 s_button_driver[i].max_mv);
        ret |= iot_button_create(&button_config,
                                 &s_button_driver[i].base,
                                 &btn_array[i]);
    }

    if (btn_cnt != NULL) {
        *btn_cnt = BSP_BUTTON_NUM;
    }
    return ret;
}
