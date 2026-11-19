#include "device_auth.h"

#include <string.h>

static const char AUTH_ALPHABET[] =
    "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";

static int auth_alphabet_index(char value)
{
    const char *found = strchr(AUTH_ALPHABET, value);
    return found == NULL ? -1 : (int)(found - AUTH_ALPHABET);
}

void device_auth_generate_code(const uint8_t entropy[8],
                               char *out,
                               size_t capacity)
{
    if (out == NULL || capacity < TD_AUTH_CODE_LEN) {
        return;
    }
    if (entropy == NULL) {
        memset(out, 0, capacity);
        return;
    }
    for (size_t i = 0u; i < 8u; ++i) {
        size_t index = i < 4u ? i : i + 1u;
        out[index] = AUTH_ALPHABET[entropy[i] & 0x1Fu];
    }
    out[4] = '-';
    out[9] = '\0';
}

bool device_auth_code_valid(const char *code)
{
    if (code == NULL || strlen(code) != 9u || code[4] != '-') {
        return false;
    }
    for (size_t i = 0u; i < 9u; ++i) {
        if (i == 4u) {
            continue;
        }
        if (auth_alphabet_index(code[i]) < 0) {
            return false;
        }
    }
    return true;
}

bool device_auth_code_matches(const char *expected, const char *candidate)
{
    uint8_t difference = 0u;

    if (!device_auth_code_valid(expected) ||
        !device_auth_code_valid(candidate)) {
        return false;
    }
    for (size_t i = 0u; i < 9u; ++i) {
        difference |= (uint8_t)(expected[i] ^ candidate[i]);
    }
    return difference == 0u;
}
