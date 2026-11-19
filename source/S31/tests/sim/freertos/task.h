#pragma once
#include "FreeRTOS.h"
static inline int xTaskCreate(void (*entry)(void *), const char *name,
                             unsigned stack, void *arg, unsigned priority, void *handle)
{
    (void)entry; (void)name; (void)stack; (void)arg; (void)priority; (void)handle;
    return pdPASS;
}
static inline void vTaskDelay(TickType_t ticks) { (void)ticks; }
