#include "voice_player.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "bsp/esp-bsp.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "voice_wav.h"
#include "driver/uart.h"
#include "diagnostics/startup_selftest.h"

#define VOICE_PARTITION "voice"
#define VOICE_MOUNT "/voice"
#define VOICE_NAME_LEN 24
static const char *TAG = "voice";
static esp_codec_dev_handle_t s_spk;
static SemaphoreHandle_t s_mutex;
static QueueHandle_t s_queue;
static bool s_codec_open;
static voice_wav_info_t s_format;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static voice_status_t s_status;

typedef struct {
    char clip[VOICE_NAME_LEN];
    uint16_t face_id; /* Zero denotes a single WAV clip. */
} voice_job_t;

static esp_err_t play_greeting(uint16_t face_id);

/* Physical UART diagnostics only; this console accepts no motion commands. */
static void voice_console_task(void *arg)
{
    (void)arg;
    char command[64];
    size_t length = 0;
    bool overflow = false;
    while (true) {
        char ch;
        if (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &ch, 1,
                            pdMS_TO_TICKS(100)) != 1) continue;
        if (ch == '\r' || ch == '\n') {
            command[length] = '\0';
            if (!overflow && !strcmp(command, "voice_status")) {
                voice_status_t status;
                voice_get_status(&status);
                ESP_LOGI(TAG, "status ready=%d playing=%d completed=%lu error=%s clip=%s",
                         status.ready, status.playing, (unsigned long)status.completed,
                         esp_err_to_name(status.last_error), status.clip);
            } else if (!overflow && !strcmp(command, "selftest_status")) {
                startup_selftest_log();
            } else if (!overflow && !strncmp(command, "voice ", 6)) {
                esp_err_t err = voice_play_async(command + 6);
                ESP_LOGI(TAG, "queue result=%s", esp_err_to_name(err));
            }
            length = 0;
            overflow = false;
        } else if (length < sizeof(command) - 1u) {
            command[length++] = ch;
        } else {
            overflow = true;
        }
    }
}

void voice_get_status(voice_status_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_status_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_status_lock);
}

static void record_result(esp_err_t err, bool playing, bool completed)
{
    portENTER_CRITICAL(&s_status_lock);
    s_status.last_error = err;
    s_status.playing = playing;
    if (completed) ++s_status.completed;
    portEXIT_CRITICAL(&s_status_lock);
}

static bool valid_name(const char *name)
{
    if (!name || !*name || strlen(name) >= VOICE_NAME_LEN) return false;
    for (const char *p = name; *p; ++p) {
        if (!((*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || *p == '_')) return false;
    }
    return true;
}

static esp_err_t open_clip(const char *name, FILE **file, voice_wav_info_t *info)
{
    char path[64];
    if (!valid_name(name)) return ESP_ERR_INVALID_ARG;
    snprintf(path, sizeof(path), VOICE_MOUNT "/%s.wav", name);
    int saved_errno = 0;
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        *file = fopen(path, "rb");
        if (*file) break;
        saved_errno = errno;
        /* A flash read can transiently fail during model/TLS memory peaks. */
        if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!*file) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", path, saved_errno);
        return saved_errno == ENOMEM ? ESP_ERR_NO_MEM :
               saved_errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }
    if (voice_wav_parse(*file, info) != 0) {
        fclose(*file);
        *file = NULL;
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

static esp_err_t configure_codec(const voice_wav_info_t *info)
{
    if (s_codec_open && (s_format.sample_rate != info->sample_rate ||
        s_format.channels != info->channels || s_format.bits != info->bits)) {
        int err = esp_codec_dev_close(s_spk);
        if (err != ESP_OK) return ESP_FAIL;
        s_codec_open = false;
    }
    if (!s_codec_open) {
        esp_codec_dev_sample_info_t format = {
            .sample_rate = (int)info->sample_rate,
            .channel = 2,
            .bits_per_sample = info->bits,
        };
        if (esp_codec_dev_open(s_spk, &format) != ESP_OK) return ESP_FAIL;
        s_codec_open = true;
        s_format = *info;
    }
    /* Apply volume after the codec is open, and verify unmute. */
    if (esp_codec_dev_set_out_vol(s_spk, 75) != ESP_OK ||
        esp_codec_dev_set_out_mute(s_spk, false) != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

bool voice_ready(void)
{
    voice_status_t status;
    voice_get_status(&status);
    return status.ready;
}

static void voice_task(void *arg)
{
    (void)arg;
    voice_job_t job;
    while (xQueueReceive(s_queue, &job, portMAX_DELAY) == pdTRUE) {
        esp_err_t err = job.face_id ? play_greeting(job.face_id) : voice_play(job.clip);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "voice job failed: %s", esp_err_to_name(err));
        }
    }
    vTaskDelete(NULL);
}

esp_err_t voice_init(void)
{
    if (voice_ready()) return ESP_OK;
    esp_vfs_spiffs_conf_t conf = {
        .base_path = VOICE_MOUNT, .partition_label = VOICE_PARTITION,
        .max_files = 6, .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) goto failed;
    FILE *test = NULL;
    voice_wav_info_t info;
    err = open_clip("say_test", &test, &info);
    if (err != ESP_OK) goto failed;
    fclose(test);
    if (!s_spk) s_spk = bsp_audio_codec_speaker_init();
    if (!s_spk) { err = ESP_FAIL; goto failed; }
    err = configure_codec(&info);
    if (err != ESP_OK) goto failed;
    s_mutex = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(4, sizeof(voice_job_t));
    if (!s_mutex || !s_queue) { err = ESP_ERR_NO_MEM; goto cleanup; }
    portENTER_CRITICAL(&s_status_lock);
    s_status.ready = true;
    s_status.last_error = ESP_OK;
    portEXIT_CRITICAL(&s_status_lock);
    if (xTaskCreatePinnedToCore(voice_task, "voice", 6144, NULL, 5, NULL, 1) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    ESP_LOGI(TAG, "voice ready: test clip and speaker codec verified");
    if (uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 0, 0, NULL, 0) == ESP_OK) {
        if (xTaskCreate(voice_console_task, "voice_console", 3072, NULL, 2, NULL) != pdPASS) {
            uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);
            ESP_LOGW(TAG, "voice UART diagnostics unavailable");
        }
    }
    return ESP_OK;
cleanup:
    portENTER_CRITICAL(&s_status_lock);
    s_status.ready = false;
    portEXIT_CRITICAL(&s_status_lock);
    if (s_queue) { vQueueDelete(s_queue); s_queue = NULL; }
    if (s_mutex) { vSemaphoreDelete(s_mutex); s_mutex = NULL; }
failed:
    if (s_codec_open) { esp_codec_dev_close(s_spk); s_codec_open = false; }
    record_result(err, false, false);
    ESP_LOGE(TAG, "voice init failed: %s (flash the voice partition too)", esp_err_to_name(err));
    return err;
}

esp_err_t voice_play_async(const char *name)
{
    if (!valid_name(name)) return ESP_ERR_INVALID_ARG;
    if (!voice_ready() || !s_queue) return ESP_ERR_INVALID_STATE;
    FILE *file = NULL;
    voice_wav_info_t info;
    esp_err_t err = open_clip(name, &file, &info);
    if (err != ESP_OK) { record_result(err, false, false); return err; }
    fclose(file);
    voice_job_t job = {0};
    strlcpy(job.clip, name, sizeof(job.clip));
    return xQueueSend(s_queue, &job, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t voice_greet_async(uint16_t face_id)
{
    if (face_id == 0) return ESP_ERR_INVALID_ARG;
    if (!voice_ready() || !s_queue) return ESP_ERR_INVALID_STATE;
    startup_report_t selftest;
    startup_selftest_snapshot(&selftest);
    if (selftest.announcing || uxQueueMessagesWaiting(s_queue) != 0u) {
        return ESP_ERR_TIMEOUT;
    }
    const voice_job_t job = {.face_id = face_id};
    return xQueueSend(s_queue, &job, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t play_locked(const char *name)
{
    FILE *file = NULL;
    voice_wav_info_t info;
    esp_err_t err = open_clip(name, &file, &info);
    if (err == ESP_OK) err = configure_codec(&info);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_status_lock);
        strlcpy(s_status.clip, name, sizeof(s_status.clip));
        s_status.playing = true;
        s_status.last_error = ESP_OK;
        portEXIT_CRITICAL(&s_status_lock);
        int16_t buffer[1024];
        uint32_t remain = info.data_size;
        while (remain) {
            size_t capacity = info.channels == 1u ? sizeof(buffer) / 2u : sizeof(buffer);
            size_t count = remain > capacity ? capacity : remain;
            if (fread(buffer, 1, count, file) != count) { err = ESP_ERR_INVALID_SIZE; break; }
            size_t output_bytes = count;
            if (info.channels == 1u) {
                /* A single speaker may be plugged into either L or R. */
                for (size_t i = count / 2u; i > 0u; --i) {
                    int16_t sample = buffer[i - 1u];
                    buffer[2u * (i - 1u)] = sample;
                    buffer[2u * (i - 1u) + 1u] = sample;
                }
                output_bytes *= 2u;
            }
            if (esp_codec_dev_write(s_spk, buffer, (int)output_bytes) != ESP_OK) { err = ESP_FAIL; break; }
            remain -= (uint32_t)count;
        }
        if (err == ESP_OK) ESP_LOGI(TAG, "played %s: %lu PCM bytes", name, (unsigned long)info.data_size);
    }
    if (file) fclose(file);
    record_result(err, false, err == ESP_OK);
    return err;
}

/* Hold the codec lock across the sentence so digit clips stay together. */
static esp_err_t play_greeting(uint16_t face_id)
{
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_INVALID_STATE;
    char digits[6];
    snprintf(digits, sizeof(digits), "%03u", (unsigned)face_id);
    esp_err_t err = play_locked("greet_hello");
    for (const char *digit = digits; *digit && err == ESP_OK; ++digit) {
        char clip[] = "greet_d0";
        clip[7] = *digit;
        err = play_locked(clip);
    }
    if (err == ESP_OK) {
        err = play_locked("greet_wish");
    }
    xSemaphoreGive(s_mutex);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "greeted face %03u", (unsigned)face_id);
    }
    return err;
}

esp_err_t voice_play(const char *name)
{
    if (!valid_name(name)) return ESP_ERR_INVALID_ARG;
    if (!voice_ready() || !s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_INVALID_STATE;
    esp_err_t err = play_locked(name);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t voice_play_pair(const char *first, const char *second)
{
    if (!valid_name(first) || !valid_name(second)) return ESP_ERR_INVALID_ARG;
    if (!voice_ready() || !s_mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_INVALID_STATE;
    esp_err_t err = play_locked(first);
    if (err == ESP_OK) err = play_locked(second);
    xSemaphoreGive(s_mutex);
    return err;
}
