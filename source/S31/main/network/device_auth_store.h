#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t device_auth_store_init(void);
bool device_auth_store_ready(void);
const char *device_auth_control_code(void);
const char *device_auth_ap_password(void);
