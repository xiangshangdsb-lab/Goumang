#include "device_auth_store.h"

#include <string.h>

#include "device_auth.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs.h"
#include "nvs_flash.h"

#define AUTH_NVS_NAMESPACE "td_sec"
#define AUTH_NVS_CONTROL   "control"
#define AUTH_NVS_AP_PASS   "ap_pass"

static const char *TAG = "device_auth";
static char s_control_code[TD_AUTH_CODE_LEN];
static char s_ap_password[16];
static bool s_ready;

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    return err;
}

static void generate_values(void)
{
    uint8_t entropy[8];

    esp_fill_random(entropy, sizeof(entropy));
    device_auth_generate_code(entropy, s_control_code,
                              sizeof(s_control_code));
    esp_fill_random(entropy, sizeof(entropy));
    strlcpy(s_ap_password, "TD-", sizeof(s_ap_password));
    char ap_code[TD_AUTH_CODE_LEN];
    device_auth_generate_code(entropy, ap_code, sizeof(ap_code));
    strlcat(s_ap_password, ap_code, sizeof(s_ap_password));
}

static esp_err_t load_or_create(nvs_handle_t handle)
{
    size_t length = sizeof(s_control_code);
    esp_err_t err = nvs_get_str(handle, AUTH_NVS_CONTROL,
                                s_control_code, &length);
    bool changed = false;

    if (err != ESP_OK || !device_auth_code_valid(s_control_code)) {
        uint8_t entropy[8];
        esp_fill_random(entropy, sizeof(entropy));
        device_auth_generate_code(entropy, s_control_code,
                                  sizeof(s_control_code));
        changed = true;
    }
    length = sizeof(s_ap_password);
    err = nvs_get_str(handle, AUTH_NVS_AP_PASS,
                      s_ap_password, &length);
    if (err != ESP_OK || strlen(s_ap_password) < 12u) {
        uint8_t entropy[8];
        esp_fill_random(entropy, sizeof(entropy));
        strlcpy(s_ap_password, "TD-", sizeof(s_ap_password));
        char ap_code[TD_AUTH_CODE_LEN];
        device_auth_generate_code(entropy, ap_code, sizeof(ap_code));
        strlcat(s_ap_password, ap_code, sizeof(s_ap_password));
        changed = true;
    }
    if (changed) {
        err = nvs_set_str(handle, AUTH_NVS_CONTROL, s_control_code);
        if (err == ESP_OK) {
            err = nvs_set_str(handle, AUTH_NVS_AP_PASS, s_ap_password);
        }
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
    } else {
        err = ESP_OK;
    }
    return err;
}

esp_err_t device_auth_store_init(void)
{
    nvs_handle_t handle;
    esp_err_t err = init_nvs();

    if (err != ESP_OK) {
        generate_values();
        s_ready = true;
        return err;
    }
    err = nvs_open(AUTH_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        generate_values();
        s_ready = true;
        return err;
    }
    err = load_or_create(handle);
    nvs_close(handle);
    s_ready = true;
    ESP_LOGI(TAG, "device control code initialized");
    return err;
}

bool device_auth_store_ready(void)
{
    return s_ready;
}

const char *device_auth_control_code(void)
{
    return s_control_code;
}

const char *device_auth_ap_password(void)
{
    return s_ap_password;
}
