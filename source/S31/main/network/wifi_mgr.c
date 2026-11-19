#include "wifi_mgr.h"

#include <string.h>

#include "app/app_config.h"
#include "app/app_state.h"
#include "device_auth_store.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "wifi";

#define WIFI_NVS_NAMESPACE       "td_net"
#define WIFI_NVS_KEY_MODE        "mode"
#define WIFI_NVS_KEY_SSID        "ssid"
#define WIFI_NVS_KEY_PASSWORD    "password"
#define WIFI_STA_FALLBACK_US     (20ll * 1000ll * 1000ll)
#define WIFI_STA_RETRY_INTERVAL_US (3ll * 1000ll * 1000ll)
#define WIFI_AP_IP               "192.168.4.1"

static SemaphoreHandle_t s_mutex;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static bool s_wifi_started;
static bool s_sta_connected;
static bool s_sta_has_ip;
static bool s_ap_active;
static bool s_ap_client_connected;
static bool s_setup_ap;
static bool s_fallback_pending;
static bool s_scanning;
static uint8_t s_preferred_mode = TD_WIFI_MODE_STATION;
static int64_t s_sta_disconnected_us;
static int64_t s_sta_retry_due_us;
static char s_station_ssid[33];
static char s_station_password[64];
static char s_current_ip[16] = WIFI_AP_IP;
static char s_current_url[32] = TD_AP_URL;
static bool s_sntp_started;

static void start_time_sync(void)
{
    if (s_sntp_started) {
        return;
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0u, "ntp.aliyun.com");
    esp_sntp_init();
    s_sntp_started = true;
}

static void publish_state(void)
{
    td_wifi_status_t status = {0};
    wifi_mgr_get_status(&status);
    app_state_lock();
    td_status_t *app = app_state_mutable();
    app->sta_connected = status.station_connected;
    app->ap_started = status.access_point_active;
    app->setup_ap = status.setup_access_point;
    app->ap_client_connected = status.access_point_client_connected;
    app->wifi_preferred_mode = status.preferred_mode;
    strlcpy(app->ap_ip, status.ip, sizeof(app->ap_ip));
    strlcpy(app->url, status.url, sizeof(app->url));
    app_state_unlock();
}

static esp_err_t load_settings(void)
{
    nvs_handle_t handle;
    size_t length;
    uint8_t mode = TD_WIFI_MODE_STATION;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    (void)nvs_get_u8(handle, WIFI_NVS_KEY_MODE, &mode);
    if (mode == TD_WIFI_MODE_ACCESS_POINT ||
        mode == TD_WIFI_MODE_STATION) {
        s_preferred_mode = mode;
    }
    length = sizeof(s_station_ssid);
    if (nvs_get_str(handle, WIFI_NVS_KEY_SSID, s_station_ssid,
                    &length) != ESP_OK) {
        s_station_ssid[0] = '\0';
    }
    length = sizeof(s_station_password);
    if (nvs_get_str(handle, WIFI_NVS_KEY_PASSWORD, s_station_password,
                    &length) != ESP_OK) {
        s_station_password[0] = '\0';
    }
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t persist_settings(void)
{
    nvs_handle_t handle;
    esp_err_t err =
        nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(handle, WIFI_NVS_KEY_MODE, s_preferred_mode);
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_KEY_SSID, s_station_ssid);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, WIFI_NVS_KEY_PASSWORD,
                          s_station_password);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static void set_ip_state(const char *ip)
{
    strlcpy(s_current_ip, ip, sizeof(s_current_ip));
    if (s_sta_has_ip && ip != NULL && ip[0] != '\0') {
        snprintf(s_current_url, sizeof(s_current_url), "http://%s", ip);
    } else if (ip != NULL && strcmp(ip, WIFI_AP_IP) == 0) {
        strlcpy(s_current_url, TD_AP_URL, sizeof(s_current_url));
    } else {
        s_current_url[0] = '\0';
    }
}

static uint8_t station_password_mode(const char *password)
{
    size_t length = password == NULL ? 0u : strlen(password);
    if (length == 0u) {
        return WIFI_AUTH_OPEN;
    }
    if (length < 8u) {
        return WIFI_AUTH_WPA2_PSK;
    }
    return WIFI_AUTH_WPA2_PSK;
}

static esp_err_t start_station(bool keep_access_point)
{
    wifi_config_t config = {0};
    esp_err_t err;

    if (s_station_ssid[0] == '\0') {
        return ESP_ERR_INVALID_STATE;
    }
    strlcpy((char *)config.sta.ssid, s_station_ssid,
            sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, s_station_password,
            sizeof(config.sta.password));
    config.sta.threshold.authmode =
        station_password_mode(s_station_password);
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;

    err = esp_wifi_set_mode(
        keep_access_point ? WIFI_MODE_APSTA : WIFI_MODE_STA);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_STA, &config);
    }
    if (err == ESP_OK) {
        s_fallback_pending = true;
        s_sta_disconnected_us = esp_timer_get_time();
        s_sta_retry_due_us =
            s_sta_disconnected_us + WIFI_STA_RETRY_INTERVAL_US;
        s_sta_connected = false;
        s_sta_has_ip = false;
        s_setup_ap = false;
        set_ip_state(s_ap_active ? WIFI_AP_IP : "");
        err = esp_wifi_connect();
        if (err == ESP_ERR_WIFI_CONN) {
            err = ESP_OK;
        }
    }
    publish_state();
    return err;
}

static esp_err_t start_access_point(bool setup)
{
    wifi_config_t config = {0};
    esp_err_t err;
    const char *ap_password = device_auth_ap_password();

    strlcpy((char *)config.ap.ssid,
            setup ? TD_AP_SETUP_SSID : TD_AP_SSID,
            sizeof(config.ap.ssid));
    config.ap.ssid_len = (uint8_t)strlen((char *)config.ap.ssid);
    config.ap.channel = TD_AP_CHANNEL;
    config.ap.max_connection = TD_AP_MAX_CLIENTS;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.required = false;
    if (!device_auth_store_ready() || ap_password == NULL ||
        strlen(ap_password) < 8u) {
        return ESP_ERR_INVALID_STATE;
    }
    strlcpy((char *)config.ap.password, ap_password,
            sizeof(config.ap.password));

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &config);
    }
    if (err == ESP_OK) {
        s_setup_ap = setup;
        s_fallback_pending = setup;
        s_sta_connected = false;
        s_sta_has_ip = false;
        s_sta_retry_due_us = 0;
        if (!s_wifi_started) {
            err = esp_wifi_start();
            s_wifi_started = err == ESP_OK;
        }
        set_ip_state(WIFI_AP_IP);
    }
    publish_state();
    return err;
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t id,
                               void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_scanning) {
            return;
        }
        s_fallback_pending = true;
        s_sta_disconnected_us = esp_timer_get_time();
        s_sta_retry_due_us =
            s_sta_disconnected_us + WIFI_STA_RETRY_INTERVAL_US;
        s_sta_has_ip = false;
        if (!s_scanning) {
            (void)esp_wifi_connect();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        s_sta_connected = true;
        s_sta_has_ip = false;
        s_sta_retry_due_us = 0;
        ESP_LOGI(TAG, "station link established; waiting for IP");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        int64_t now = esp_timer_get_time();
        s_sta_connected = false;
        s_sta_has_ip = false;
        /* 连续尝试同一路由器时保留首次掉线时间，让 20 秒回退能触发。 */
        if (!s_fallback_pending) {
            s_sta_disconnected_us = now;
        }
        s_fallback_pending = true;
        s_sta_retry_due_us = now + WIFI_STA_RETRY_INTERVAL_US;
        set_ip_state(s_ap_active ? WIFI_AP_IP : "");
        ESP_LOGW(TAG, "station disconnected; retry/fallback armed");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        s_ap_active = true;
        set_ip_state(s_sta_has_ip ? s_current_ip : WIFI_AP_IP);
        ESP_LOGI(TAG, "access point active (%s)",
                 s_setup_ap ? "setup" : "manual");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STOP) {
        s_ap_active = false;
        s_ap_client_connected = false;
        if (!s_sta_has_ip) {
            set_ip_state("");
        }
    } else if (base == WIFI_EVENT &&
               id == WIFI_EVENT_AP_STACONNECTED) {
        s_ap_client_connected = true;
    } else if (base == WIFI_EVENT &&
               id == WIFI_EVENT_AP_STADISCONNECTED) {
        s_ap_client_connected = false;
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        char ip[16];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip, sizeof(ip));
        s_sta_connected = true;
        s_sta_has_ip = true;
        s_fallback_pending = false;
        s_sta_retry_due_us = 0;
        set_ip_state(ip);
        start_time_sync();
        ESP_LOGI(TAG, "station IP=%s", ip);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        s_sta_connected = false;
        s_sta_has_ip = false;
        int64_t now = esp_timer_get_time();
        if (!s_fallback_pending) {
            s_sta_disconnected_us = now;
        }
        s_fallback_pending = true;
        s_sta_retry_due_us = now + WIFI_STA_RETRY_INTERVAL_US;
        set_ip_state(s_ap_active ? WIFI_AP_IP : "");
    }
    publish_state();
}

static void wifi_supervisor_task(void *arg)
{
    (void)arg;
    while (true) {
        bool switch_to_ap = false;
        bool close_ap = false;
        bool retry_station = false;
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            int64_t now = esp_timer_get_time();
            retry_station =
                s_fallback_pending && !s_sta_connected &&
                s_preferred_mode == TD_WIFI_MODE_STATION &&
                !s_ap_active && !s_scanning &&
                s_station_ssid[0] != '\0' &&
                s_sta_retry_due_us != 0 &&
                now >= s_sta_retry_due_us;
            if (s_fallback_pending && !s_sta_has_ip &&
                s_preferred_mode == TD_WIFI_MODE_STATION &&
                !s_ap_active &&
                now - s_sta_disconnected_us >= WIFI_STA_FALLBACK_US) {
                switch_to_ap = true;
            }
            if (s_sta_has_ip && s_ap_active &&
                s_preferred_mode == TD_WIFI_MODE_STATION) {
                close_ap = true;
            }
            if (retry_station && !switch_to_ap) {
                esp_err_t retry_err = esp_wifi_connect();
                if (retry_err != ESP_OK &&
                    retry_err != ESP_ERR_WIFI_CONN) {
                    ESP_LOGW(TAG, "station retry failed: %s",
                             esp_err_to_name(retry_err));
                }
                s_sta_retry_due_us =
                    now + WIFI_STA_RETRY_INTERVAL_US;
            }
            if (switch_to_ap) {
                (void)start_access_point(true);
            } else if (close_ap) {
                (void)esp_wifi_set_mode(WIFI_MODE_STA);
            }
            xSemaphoreGive(s_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t wifi_mgr_start(void)
{
    esp_err_t err;

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase NVS");
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }
    ESP_RETURN_ON_ERROR(load_settings(), TAG, "load Wi-Fi settings");

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_sta_netif == NULL || s_ap_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }
    (void)esp_netif_set_hostname(s_sta_netif, TD_MDNS_HOSTNAME);

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config),
                        TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM),
                        TAG, "wifi RAM storage");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   wifi_event_handler, NULL),
        TAG, "register Wi-Fi event handler");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                   wifi_event_handler, NULL),
        TAG, "register IP event handler");

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        if (s_preferred_mode == TD_WIFI_MODE_ACCESS_POINT) {
            err = start_access_point(false);
        } else if (s_station_ssid[0] != '\0') {
            err = esp_wifi_set_mode(WIFI_MODE_STA);
            if (err == ESP_OK) {
                err = esp_wifi_start();
                s_wifi_started = err == ESP_OK;
            }
            if (err == ESP_OK) {
                err = start_station(false);
            }
        } else {
            err = start_access_point(true);
        }
        xSemaphoreGive(s_mutex);
    } else {
        err = ESP_ERR_TIMEOUT;
    }
    if (err != ESP_OK) {
        return err;
    }
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    (void)esp_wifi_set_max_tx_power(52);
    if (xTaskCreate(wifi_supervisor_task, "wifi_supervisor", 4096,
                    NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void wifi_mgr_get_status(td_wifi_status_t *status)
{
    if (status == NULL) {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->station_connected = s_sta_has_ip;
    status->access_point_active = s_ap_active;
    status->setup_access_point = s_setup_ap;
    status->access_point_client_connected = s_ap_client_connected;
    status->preferred_mode = s_preferred_mode;
    strlcpy(status->ip, s_current_ip, sizeof(status->ip));
    strlcpy(status->url, s_current_url, sizeof(status->url));
    strlcpy(status->ssid, s_station_ssid, sizeof(status->ssid));
}

esp_err_t wifi_mgr_connect_station(const char *ssid, const char *password)
{
    size_t ssid_length;
    size_t password_length;
    bool keep_ap;
    esp_err_t err;

    if (ssid == NULL || password == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ssid_length = strlen(ssid);
    password_length = strlen(password);
    if (ssid_length == 0u || ssid_length > 32u ||
        (password_length != 0u &&
         (password_length < 8u || password_length > 63u))) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    strlcpy(s_station_ssid, ssid, sizeof(s_station_ssid));
    strlcpy(s_station_password, password, sizeof(s_station_password));
    s_preferred_mode = TD_WIFI_MODE_STATION;
    err = persist_settings();
    if (err == ESP_OK) {
        keep_ap = s_ap_active;
        if (!s_wifi_started) {
            err = esp_wifi_start();
            s_wifi_started = err == ESP_OK;
        }
        if (err == ESP_OK) {
            err = start_station(keep_ap);
        }
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t wifi_mgr_set_mode(uint8_t mode)
{
    esp_err_t err = ESP_OK;
    if (mode != TD_WIFI_MODE_STATION &&
        mode != TD_WIFI_MODE_ACCESS_POINT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_preferred_mode = mode;
    err = persist_settings();
    if (err == ESP_OK) {
        if (mode == TD_WIFI_MODE_ACCESS_POINT) {
            err = start_access_point(false);
        } else if (s_station_ssid[0] != '\0') {
            err = start_station(s_ap_active);
        } else {
            err = start_access_point(true);
        }
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t wifi_mgr_forget_station(void)
{
    esp_err_t err;
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_station_ssid[0] = '\0';
    s_station_password[0] = '\0';
    s_preferred_mode = TD_WIFI_MODE_STATION;
    err = persist_settings();
    if (err == ESP_OK) {
        (void)esp_wifi_disconnect();
        err = start_access_point(true);
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t wifi_mgr_start_setup_portal(void)
{
    esp_err_t err;
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (!s_wifi_started) {
        err = esp_wifi_start();
        s_wifi_started = err == ESP_OK;
    } else {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = start_access_point(true);
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t wifi_mgr_scan(wifi_ap_record_t *records, uint16_t capacity,
                        uint16_t *count)
{
    wifi_scan_config_t config = {0};
    uint16_t found = 0u;
    bool restore_ap;
    esp_err_t err;

    if (records == NULL || capacity == 0u || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *count = 0u;
    if (s_mutex == NULL ||
        xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    restore_ap = s_ap_active && !s_sta_connected;
    s_scanning = true;
    if (restore_ap) {
        err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    } else {
        err = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (err == ESP_OK) {
        err = esp_wifi_scan_start(&config, true);
    }
    if (err == ESP_OK) {
        found = capacity;
        err = esp_wifi_scan_get_ap_records(&found, records);
        if (err == ESP_OK) {
            *count = found;
        }
    }
    wifi_mode_t restore_mode =
        (s_ap_active &&
         s_preferred_mode == TD_WIFI_MODE_STATION &&
         s_station_ssid[0] != '\0') ? WIFI_MODE_APSTA :
        s_ap_active ? WIFI_MODE_AP : WIFI_MODE_STA;
    (void)esp_wifi_set_mode(restore_mode);
    s_scanning = false;
    if (!s_sta_connected &&
        s_preferred_mode == TD_WIFI_MODE_STATION &&
        s_station_ssid[0] != '\0') {
        (void)esp_wifi_connect();
        s_sta_retry_due_us =
            esp_timer_get_time() + WIFI_STA_RETRY_INTERVAL_US;
    }
    xSemaphoreGive(s_mutex);
    return err;
}
