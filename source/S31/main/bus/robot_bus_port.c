/*
 * robot_bus_port.c - ESP-IDF port for robot_bus.
 *
 * Only three symbols live here: the recursive lock and the millisecond clock.
 * The bus holds that lock across user callbacks (topic subscribers, service
 * handlers, action steps, param apply hooks), and those callbacks are allowed
 * to call back into the bus, so the mutex MUST be recursive.  A plain mutex
 * would self-deadlock on the first nested call.
 *
 * Host test builds must NOT link this file: source/S31/tests/test_bus.c
 * provides counting no-op implementations of rb_port_lock/unlock and a
 * controllable rb_port_now_ms().
 *
 * rb_init() is expected to run from app_main before the worker tasks touch the
 * bus; the first rb_port_lock() creates the mutex lazily.
 */

#include "robot_bus.h"

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "robot_bus";

static SemaphoreHandle_t s_mutex = NULL;

void rb_port_lock(void)
{
    if (s_mutex == NULL) {
        /* Created on first use (rb_init from app_main).  The bus is not
         * touched by multiple tasks before rb_init, so no extra guard is
         * needed here; a second creation would only leak the first mutex. */
        s_mutex = xSemaphoreCreateRecursiveMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "recursive mutex allocation failed");
            return;
        }
    }
    if (xSemaphoreTakeRecursive(s_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "rb_port_lock failed");
    }
}

void rb_port_unlock(void)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreGiveRecursive(s_mutex) != pdTRUE) {
        ESP_LOGE(TAG, "rb_port_unlock without matching lock");
    }
}

uint32_t rb_port_now_ms(void)
{
    int64_t us = esp_timer_get_time();

    if (us < 0) {
        us = 0;
    }
    return (uint32_t)((uint64_t)us / 1000u);
}