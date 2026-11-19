#include <stdio.h>
#include <string.h>

#include "network/device_auth.h"

static int checks;
static int failures;

#define CHECK(value)                                                     \
    do {                                                                 \
        ++checks;                                                        \
        if (!(value)) {                                                  \
            ++failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #value);      \
        }                                                                \
    } while (0)

int main(void)
{
    const uint8_t entropy[8] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
    char code[TD_AUTH_CODE_LEN];

    device_auth_generate_code(entropy, code, sizeof(code));
    CHECK(strcmp(code, "ABCD-EFGH") == 0);
    CHECK(device_auth_code_valid(code));
    CHECK(device_auth_code_matches(code, "ABCD-EFGH"));
    CHECK(!device_auth_code_matches(code, "ABCD-EFGI"));
    CHECK(!device_auth_code_matches(code, "abcd-efgh"));
    CHECK(!device_auth_code_matches(NULL, code));
    CHECK(!device_auth_code_matches(code, NULL));
    CHECK(!device_auth_code_valid("ABCDEFGH"));
    CHECK(!device_auth_code_valid("ABCD-EFG"));
    CHECK(!device_auth_code_valid("ABCD-EFGHX"));
    printf("checks=%d failures=%d\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
