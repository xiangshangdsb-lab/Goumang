#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi.h"

enum {
    TD_WIFI_MODE_STATION = 0,
    TD_WIFI_MODE_ACCESS_POINT = 1,
};

typedef struct {
    bool station_connected;
    bool access_point_active;
    bool setup_access_point;
    bool access_point_client_connected;
    uint8_t preferred_mode;
    char ip[16];
    char url[32];
    char ssid[33];
} td_wifi_status_t;

esp_err_t wifi_mgr_start(void);
void wifi_mgr_get_status(td_wifi_status_t *status);
esp_err_t wifi_mgr_connect_station(const char *ssid, const char *password);
esp_err_t wifi_mgr_set_mode(uint8_t mode);
esp_err_t wifi_mgr_start_setup_portal(void);
esp_err_t wifi_mgr_forget_station(void);
esp_err_t wifi_mgr_scan(wifi_ap_record_t *records, uint16_t capacity,
                        uint16_t *count);
