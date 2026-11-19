#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define TD_AGENT_REPLY_CAP       2048u
#define TD_AGENT_ERROR_CAP       128u
#define TD_AGENT_TOOL_CAP        128u
#define TD_AGENT_MESSAGE_CAP     768u

typedef struct {
    bool configured;
    bool busy;
    bool done;
    uint32_t sequence;
    uint32_t requests;
    char reply[TD_AGENT_REPLY_CAP];
    char error[TD_AGENT_ERROR_CAP];
    char tool_summary[TD_AGENT_TOOL_CAP];
} td_agent_status_t;

esp_err_t agent_service_start(void);
esp_err_t agent_service_set_api_key(const char *key);
bool agent_service_is_configured(void);
esp_err_t agent_service_submit(const char *message);
void agent_service_snapshot(td_agent_status_t *status);

