#include "voice_wav.h"
#include <limits.h>
#include <stdbool.h>
#include <string.h>

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int voice_wav_parse(FILE *file, voice_wav_info_t *info)
{
    uint8_t header[12];
    bool have_fmt = false;
    long file_size;
    if (!file || !info || fseek(file, 0, SEEK_END) != 0) return -1;
    file_size = ftell(file);
    if (file_size < 12 || fseek(file, 0, SEEK_SET) != 0 ||
        fread(header, 1, 12, file) != 12 ||
        memcmp(header, "RIFF", 4) || memcmp(header + 8, "WAVE", 4)) return -1;
    uint64_t riff_end = (uint64_t)le32(header + 4) + 8u;
    if (riff_end < 12u || riff_end > (uint64_t)file_size) return -1;
    memset(info, 0, sizeof(*info));
    while ((uint64_t)ftell(file) + 8u <= riff_end) {
        uint8_t chunk[8];
        if (fread(chunk, 1, 8, file) != 8) return -1;
        uint32_t size = le32(chunk + 4);
        long start = ftell(file);
        uint64_t next = (uint64_t)start + size + (size & 1u);
        if (next > riff_end || next > LONG_MAX) return -1;
        if (!memcmp(chunk, "fmt ", 4)) {
            uint8_t fmt[16];
            if (size < 16u || fread(fmt, 1, 16, file) != 16 ||
                le16(fmt) != 1u) return -1;
            info->channels = le16(fmt + 2);
            info->sample_rate = le32(fmt + 4);
            info->bits = le16(fmt + 14);
            uint16_t align = (uint16_t)(info->channels * 2u);
            if ((info->channels != 1u && info->channels != 2u) ||
                info->bits != 16u ||
                (info->sample_rate != 8000u && info->sample_rate != 16000u &&
                 info->sample_rate != 44100u && info->sample_rate != 48000u) ||
                le16(fmt + 12) != align ||
                le32(fmt + 8) != info->sample_rate * align) return -1;
            have_fmt = true;
        } else if (!memcmp(chunk, "data", 4)) {
            if (!have_fmt || size == 0u || size % (info->channels * 2u)) return -1;
            info->data_size = size;
            return 0;
        }
        if (fseek(file, (long)next, SEEK_SET) != 0) return -1;
    }
    return -1;
}
