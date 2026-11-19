#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TD_AUTH_CODE_LEN 10u

void device_auth_generate_code(const uint8_t entropy[8],
                               char *out,
                               size_t capacity);
bool device_auth_code_valid(const char *code);
bool device_auth_code_matches(const char *expected, const char *candidate);
