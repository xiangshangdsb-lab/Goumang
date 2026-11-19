#pragma once
#include <stdint.h>
#include <stdio.h>

typedef struct {
    uint16_t channels;
    uint32_t sample_rate;
    uint16_t bits;
    uint32_t data_size;
} voice_wav_info_t;

/* On success leaves the stream at PCM data. Only 16-bit PCM is supported. */
int voice_wav_parse(FILE *file, voice_wav_info_t *info);
