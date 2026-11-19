#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../APP/storage/c30d_nvm.h"
#include "../APP/pan/pan_param_math.h"

/* Mirrors S31_PAN_STEPS_PER_180_DEG from s31_link.h. */
#define PAN_STEPS_PER_180_DEG 1600

static int s_checks;
static int s_failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        ++s_checks;                                                        \
        if (!(condition)) {                                                \
            ++s_failures;                                                  \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
        }                                                                  \
    } while (0)

static void test_steps_from_deg(void)
{
    CHECK(pan_param_steps_from_deg(PAN_HOME_DEG, PAN_STEPS_PER_180_DEG) == 0);
    CHECK(pan_param_steps_from_deg(0u, PAN_STEPS_PER_180_DEG) == -2400);
    CHECK(pan_param_steps_from_deg(PAN_MAX_DEG, PAN_STEPS_PER_180_DEG) == 2400);
    CHECK(pan_param_steps_from_deg(225u, PAN_STEPS_PER_180_DEG) == -400);
    CHECK(pan_param_steps_from_deg(315u, PAN_STEPS_PER_180_DEG) == 400);
    CHECK(pan_param_deg_from_steps(-2401, PAN_STEPS_PER_180_DEG) == 0);
    CHECK(pan_param_deg_from_steps(2401, PAN_STEPS_PER_180_DEG) == PAN_MAX_DEG);
}

static void test_degree_step_round_trip(void)
{
    for (uint16_t deg = 0; deg <= PAN_MAX_DEG; ++deg)
    {
        int32_t steps = pan_param_steps_from_deg(deg, PAN_STEPS_PER_180_DEG);
        CHECK(pan_param_deg_from_steps(steps, PAN_STEPS_PER_180_DEG) == deg);
    }
}

static void test_parameter_defaults_are_in_range(void)
{
    uint8_t id;

    for (id = 1u; id <= PAN_PARAM_COUNT; ++id)
    {
        int32_t def = pan_param_default(id);

        CHECK(pan_param_valid(id, def));
        CHECK(def >= pan_param_min(id));
        CHECK(def <= pan_param_max(id));
    }
    CHECK(pan_param_default(PAN_PARAM_LINE_BASE_PERCENT) == 30);
    CHECK(pan_param_default(PAN_PARAM_LINE_TURN_GAIN) == 12);
    CHECK(pan_param_default(PAN_PARAM_SPEED_KP) == 700);
    CHECK(pan_param_default(PAN_PARAM_SPEED_KI) == 700);
    CHECK(pan_param_default(PAN_PARAM_MAX_SPEED_MMS) == 350);
}

static void test_parameter_boundaries(void)
{
    /* line_base_percent: 5..40 */
    CHECK(!pan_param_valid(PAN_PARAM_LINE_BASE_PERCENT, 4));
    CHECK(pan_param_valid(PAN_PARAM_LINE_BASE_PERCENT, 5));
    CHECK(pan_param_valid(PAN_PARAM_LINE_BASE_PERCENT, 40));
    CHECK(!pan_param_valid(PAN_PARAM_LINE_BASE_PERCENT, 41));
    /* line_turn_gain: 0..60 */
    CHECK(pan_param_valid(PAN_PARAM_LINE_TURN_GAIN, 0));
    CHECK(pan_param_valid(PAN_PARAM_LINE_TURN_GAIN, 60));
    CHECK(!pan_param_valid(PAN_PARAM_LINE_TURN_GAIN, 61));
    CHECK(!pan_param_valid(PAN_PARAM_LINE_TURN_GAIN, -1));
    /* speed_kp / speed_ki: 0..5000 */
    CHECK(pan_param_valid(PAN_PARAM_SPEED_KP, 0));
    CHECK(pan_param_valid(PAN_PARAM_SPEED_KP, 5000));
    CHECK(!pan_param_valid(PAN_PARAM_SPEED_KP, 5001));
    CHECK(!pan_param_valid(PAN_PARAM_SPEED_KI, 5001));
    /* max_speed_mms: 50..350 */
    CHECK(!pan_param_valid(PAN_PARAM_MAX_SPEED_MMS, 49));
    CHECK(pan_param_valid(PAN_PARAM_MAX_SPEED_MMS, 50));
    CHECK(pan_param_valid(PAN_PARAM_MAX_SPEED_MMS, 350));
    CHECK(!pan_param_valid(PAN_PARAM_MAX_SPEED_MMS, 351));
    /* unknown ids are rejected */
    CHECK(!pan_param_valid(0u, 30));
    CHECK(!pan_param_valid((uint8_t)(PAN_PARAM_COUNT + 1u), 30));
    /* a negative value never passes even for a wide range */
    CHECK(!pan_param_valid(PAN_PARAM_SPEED_KP, -1));
}

static void fill_nvm_record(C30D_NvmRecord *record)
{
    memset(record, 0xA5, sizeof(*record));
    record->magic = C30D_NVM_MAGIC;
    record->version = C30D_NVM_VERSION;
    record->size = (uint16_t)sizeof(*record);
    record->pan_steps = -345;
    record->line_base_percent = 27u;
    record->line_turn_gain = 14u;
    record->speed_kp = 720u;
    record->speed_ki = 680u;
    record->max_speed_mms = 300u;
    record->reserved = 0u;
    c30d_nvm_set_crc(record);
}

static void test_nvm_record_layout_and_crc(void)
{
    C30D_NvmRecord record;

    CHECK(sizeof(C30D_NvmRecord) == 28u);
    CHECK(offsetof(C30D_NvmRecord, crc) == 24u);
    CHECK(c30d_nvm_crc_length() == 24u);

    fill_nvm_record(&record);
    CHECK(c30d_nvm_valid(&record));
    CHECK(record.pan_steps == -345);
    CHECK(record.line_turn_gain == 14u);
    CHECK(record.max_speed_mms == 300u);

    /* The CRC never covers its own field: flipping it invalidates. */
    record.crc ^= 0x5Au;
    CHECK(!c30d_nvm_valid(&record));

    /* Any protected byte change invalidates. */
    fill_nvm_record(&record);
    record.speed_kp = 721u;
    CHECK(!c30d_nvm_valid(&record));
    fill_nvm_record(&record);
    record.pan_steps = 0;
    CHECK(!c30d_nvm_valid(&record));

    /* Magic/version are checked independently of the CRC. */
    fill_nvm_record(&record);
    record.version = 0x0399u;
    c30d_nvm_set_crc(&record);
    CHECK(!c30d_nvm_valid(&record));
    fill_nvm_record(&record);
    record.magic = 0xDEADBEEFu;
    c30d_nvm_set_crc(&record);
    CHECK(!c30d_nvm_valid(&record));

    fill_nvm_record(&record);
    record.version = C30D_NVM_LEGACY;
    c30d_nvm_set_crc(&record);
    CHECK(c30d_nvm_valid(&record));
    record.reserved = C30D_NVM_PAN_MOVING;
    c30d_nvm_set_crc(&record);
    CHECK(!c30d_nvm_valid(&record));
    record.version = C30D_NVM_VERSION;
    c30d_nvm_set_crc(&record);
    CHECK(c30d_nvm_valid(&record));

    /* Blank or zeroed records cannot establish the position. */
    memset(&record, 0, sizeof(record));
    CHECK(!c30d_nvm_valid(&record));
    memset(&record, 0xFF, sizeof(record));
    CHECK(!c30d_nvm_valid(&record));
}

int main(void)
{
    test_steps_from_deg();
    test_degree_step_round_trip();
    test_parameter_defaults_are_in_range();
    test_parameter_boundaries();
    test_nvm_record_layout_and_crc();
    printf("checks=%d failures=%d\n", s_checks, s_failures);
    return s_failures == 0 ? 0 : 1;
}