#include "s31_link.h"

#include <stdio.h>
#include <string.h>

#include "system.h"
#include "../storage/c30d_nvm.h"
#include "../line/line_fusion.h"
#include "../pan/pan_param_math.h"
#include "stmflash.h"
#include "tsl1401.h"
#include "../protocol/s31_protocol.h"

#if TSL1401_PIXELS != LINE_CCD_PIXELS
#error "The CCD driver and line-fusion input must use the same sample count"
#endif

#define S31_LINE_FLAG_ACTIVE         (1u << 0)
#define S31_LINE_FLAG_CCD_FOUND      (1u << 1)
#define S31_LINE_FLAG_CCD_VALID      (1u << 2)
#define S31_LINE_FLAG_OBSTACLE       (1u << 3)
#define S31_LINE_FLAG_REWIND         (1u << 4)
#define S31_LINE_FLAG_WAIT_CLEAR     (1u << 5)

#define S31_ROBOT_IDLE               0u
#define S31_ROBOT_ACTIVE             1u
#define S31_ROBOT_ESTOP              2u

#define S31_FAULT_ESTOP              (1u << 0)
#define S31_FAULT_CMD_TIMEOUT        (1u << 1)
#define S31_FAULT_GIMBAL_UNHOMED     (1u << 2)
#define S31_FAULT_MOTION_STOP        (1u << 3)
#define S31_FAULT_IMU_OFFLINE        (1u << 4)
#define S31_FAULT_ULTRA_TIMEOUT      (1u << 5)
#define S31_FAULT_LOW_BATTERY        (1u << 6)
#define S31_FAULT_UART_OVERFLOW      (1u << 7)

#define S31_RX_RING_SIZE             256u
#define S31_FIRMWARE_MAJOR           3u
#define S31_FIRMWARE_MINOR           10u
#define S31_TEST_MAX_PERCENT         25
#define S31_TEST_DURATION_MS         3000u
#define S31_DEBUG_TELEMETRY_MS       200u

typedef struct
{
    float pwm;
    float last_bias;
} S31_PI_t;

/*
 * The original WHEELTEC project spreads these globals over system.c and
 * balance.c.  This build keeps only the symbols needed by the stripped-down
 * tracked-chassis firmware.
 */
SYS_VAL_t SysVal;
uint8_t Flag_Stop = 0u;
uint8_t Car_Mode = Tank_Car;
uint8_t Mode = 0u;
int Divisor_Mode;
int Servo;
int Led_Count = 300;
int POT_val;
float RC_Velocity = 350.0f;
float Move_X;
float Move_Y;
float Move_Z;
float Velocity_KP = S31_TRACK_SPEED_KP;
float Velocity_KI = S31_TRACK_SPEED_KI;
Smooth_Control smooth_control;
Motor_parameter MOTOR_A;
Motor_parameter MOTOR_B;
Motor_parameter MOTOR_C;
Motor_parameter MOTOR_D;
float Encoder_precision;
float Wheel_perimeter;
float Wheel_spacing;
float Axle_spacing;
float Omni_turn_radiaus;

void Robot_Init(double wheelspacing, float axlespacing,
                float omni_turn_radiaus, float gearratio,
                float accuracy, float tyre_diameter)
{
    (void)axlespacing;
    (void)omni_turn_radiaus;

    Encoder_precision = EncoderMultiples * accuracy * gearratio;
    Wheel_perimeter = tyre_diameter * PI;
    Wheel_spacing = (float)wheelspacing;
    Axle_spacing = 0.0f;
    Omni_turn_radiaus = 0.0f;
}

u32 myabs(long int value)
{
    return value < 0 ? (u32)(-value) : (u32)value;
}

static volatile uint8_t s_rx_ring[S31_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_overflow_count;
static volatile uint32_t s_estop_history;

static volatile uint8_t s_estop;
static volatile uint8_t s_armed;
static volatile uint8_t s_motion_stop_latched;
static volatile uint32_t s_bt_stop_holdoff_tick;
static volatile uint32_t s_last_drive_tick;
static volatile int8_t s_left_percent;
static volatile int8_t s_right_percent;
static volatile uint32_t s_drive_frame_count;
static volatile uint32_t s_nonzero_drive_count;
static volatile uint32_t s_last_nonzero_drive_tick;
static volatile uint8_t s_bt_armed;
static volatile int8_t s_bt_left_percent;
static volatile int8_t s_bt_right_percent;
static volatile uint32_t s_bt_last_command_tick;
static volatile uint32_t s_bt_command_count;
static volatile uint8_t s_bt_direction_action;
static volatile uint8_t s_bt_speed_percent;

/*
 * Diagnostic-only open-loop PWM. Normal motor commands still use the
 * production PI path. These values are changed only by an explicit UART1
 * command and are automatically cleared after a short, bounded test.
 */
static volatile uint8_t s_test_active;
static volatile int16_t s_test_pwm_a;
static volatile int16_t s_test_pwm_b;
static volatile uint32_t s_test_started_tick;
static volatile char s_test_code;

static uint32_t s_crc_error_count;
static s31_frame_parser_t s_parser;

static volatile uint16_t s_pan_deg;
static uint8_t s_fault_flags;
static uint8_t s_imu_ready;
static uint8_t s_ultrasonic_ready;
static uint16_t s_ultrasonic_cm;
static int16_t s_yaw_cdeg;
static uint32_t s_last_yaw_tick;
static S31_PI_t s_pi_a;
static S31_PI_t s_pi_b;

static volatile uint8_t s_pan_moving;
static volatile uint8_t s_pan_direction;
static volatile uint32_t s_pan_steps_remaining;
static volatile int32_t s_pan_current_steps;
static volatile int32_t s_pan_target_steps;
static volatile uint8_t s_pan_boot_pending;
static volatile uint8_t s_pan_boot_moving;
static int32_t s_raw_encoder_a;
static int32_t s_raw_encoder_b;
static volatile char s_main_stage;
static volatile uint32_t s_main_loop_count;
static volatile uint32_t s_uart3_txe_timeouts;
static volatile uint32_t s_uart3_tc_timeouts;
static volatile uint32_t s_battery_adc_timeouts;
static volatile uint8_t s_line_enabled;
static volatile uint8_t s_line_bt_mode;
static volatile uint8_t s_line_speed_percent = S31_LINE_BASE_PERCENT;
static volatile uint32_t s_line_last_cmd_tick;
static int8_t s_line_vision_bias;
static volatile uint8_t s_line_bits = 0x0Fu;
static volatile int8_t s_line_left_percent;
static volatile int8_t s_line_right_percent;
static line_motion_t s_line_motion;
static lf_result_t s_line_ccd;
static uint16_t s_ccd_pixels[LINE_CCD_PIXELS];
static uint32_t s_ccd_elapsed_us;
static uint32_t s_ccd_last_tick;
static uint8_t s_ccd_valid;
static uint8_t s_line_source;
static int16_t s_line_error;
static uint8_t s_line_phase;
static uint8_t s_line_rewind_progress;
static uint16_t s_ultrasonic_history[3];
static uint8_t s_ultrasonic_history_count;
static uint8_t s_ultrasonic_history_index;
static int16_t s_imu_gyro_ddps[3];
static int16_t s_imu_accel_mg[3];
static int8_t s_active_left_percent;
static int8_t s_active_right_percent;
static uint8_t s_active_source;

/* ------------------------------------------------------------------------ */
/* v3.10.0 persisted gimbal position and runtime parameter service          */

#define C30D_NVM_SAVE_DELAY_TICKS    100u           /* 100 x 10 ms = 1 s */

static int32_t s_param_values[C30D_PARAM_COUNT] = {30, 12, 700, 700, 350};
static float s_max_speed_mps = S31_MAX_WHEEL_SPEED_MPS;
static volatile u8 s_nvm_dirty;
static u16 s_nvm_idle_ticks;
static u32 s_nvm_next;
static u8 s_nvm_pan_pending;

#define C30D_NVM_END (STMFLASH_NVM_SECTOR_ADDR + STMFLASH_NVM_SECTOR_SIZE)

static void nvm_mark_dirty(void);
static int32_t param_get(u8 id);
static u8 param_set(u8 id, int32_t value);
static void nvm_load(void);
static u8 nvm_append(u16 flags, u8 slots);
static void nvm_poll(void);

static int16_t clamp_i16(int32_t value);

static void pan_enable(uint8_t enable)
{
#if S31_PAN_EN_ACTIVE_LOW
    if (enable)
    {
        GPIO_ResetBits(S31_PAN_EN_PORT, S31_PAN_EN_PIN);
    }
    else
    {
        GPIO_SetBits(S31_PAN_EN_PORT, S31_PAN_EN_PIN);
    }
#else
    if (enable)
    {
        GPIO_SetBits(S31_PAN_EN_PORT, S31_PAN_EN_PIN);
    }
    else
    {
        GPIO_ResetBits(S31_PAN_EN_PORT, S31_PAN_EN_PIN);
    }
#endif
}

static void pan_count_step(void)
{
    s_pan_current_steps += s_pan_direction != 0u ? 1 : -1;
    s_pan_deg = pan_param_deg_from_steps(s_pan_current_steps,
                                         S31_PAN_STEPS_PER_180_DEG);
    --s_pan_steps_remaining;
}

static void pan_finish_pulse(void)
{
    TIM_Cmd(TIM13, DISABLE);
    TIM_ITConfig(TIM13, TIM_IT_Update, DISABLE);
    TIM_SetCounter(TIM13, 0u);
    s_pan_target_steps = s_pan_current_steps;
    s_nvm_dirty = 1u;
    s_pan_moving = 0u;
}

static void pan_stop_pulse(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM_Cmd(TIM13, DISABLE);
    if (s_pan_moving != 0u)
    {
        /* PWM2 rises in the middle of a period. Account for a pending
         * full period and a rising edge in the current period. */
        if (TIM_GetITStatus(TIM13, TIM_IT_Update) != RESET)
        {
            pan_count_step();
        }
        if (s_pan_steps_remaining != 0u &&
            TIM_GetCounter(TIM13) >= TIM_GetCapture1(TIM13))
        {
            pan_count_step();
        }
        pan_finish_pulse();
        s_pan_steps_remaining = 0u;
        s_pan_boot_moving = 0u;
    }
    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
    pan_enable(0u);
    __set_PRIMASK(primask);
}

static void pan_gpio_timer_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef compare;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

    GPIO_PinAFConfig(S31_PAN_STEP_PORT, S31_PAN_STEP_PIN_SOURCE,
                     S31_PAN_STEP_AF);
    gpio.GPIO_Pin = S31_PAN_STEP_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(S31_PAN_STEP_PORT, &gpio);

    gpio.GPIO_Pin = S31_PAN_DIR_PIN | S31_PAN_EN_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &gpio);
    GPIO_ResetBits(S31_PAN_DIR_PORT, S31_PAN_DIR_PIN);
    pan_enable(0u);

    /*
     * APB1 timer clock is 84 MHz. Prescale to 1 MHz; 30 RPM with
     * 3200 pulses/rev gives 1600 pulses/s, i.e. a 625-tick STEP period.
     */
    time_base.TIM_Prescaler = 84u - 1u;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    time_base.TIM_Period =
        (uint16_t)(1000000u * 60u /
                   (S31_PAN_MOVE_RPM * S31_PAN_STEPS_PER_REV) - 1u);
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_RepetitionCounter = 0u;
    TIM_TimeBaseInit(TIM13, &time_base);

    compare.TIM_OCMode = TIM_OCMode_PWM2;
    compare.TIM_OutputState = TIM_OutputState_Enable;
    compare.TIM_OCPolarity = TIM_OCPolarity_High;
    compare.TIM_Pulse = (uint16_t)((time_base.TIM_Period + 1u) / 2u);
    TIM_OC1Init(TIM13, &compare);
    TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM13, ENABLE);
    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
    TIM_Cmd(TIM13, DISABLE);

    nvic.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2u;
    nvic.NVIC_IRQChannelSubPriority = 0u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

static int32_t pan_deg_to_steps(uint16_t pan_deg)
{
    return pan_param_steps_from_deg(pan_deg, S31_PAN_STEPS_PER_180_DEG);
}

static u8 pan_move_to_steps(int32_t target_steps)
{
    int32_t delta;
    uint32_t steps;
    uint16_t period;

    int32_t limit = S31_PAN_STEPS_PER_180_DEG * 3 / 2;

    if (target_steps < -limit || target_steps > limit ||
        (s_fault_flags & S31_FAULT_GIMBAL_UNHOMED) != 0u ||
        s_estop || Voltage < 10.0f || EN == 0 || Flag_Stop != 0u)
    {
        return 1u;
    }
    if (target_steps == s_pan_target_steps)
    {
        return 0u;
    }
    if (s_nvm_pan_pending == 0u)
    {
        if (nvm_append(C30D_NVM_PAN_MOVING, 2u) != 0u)
        {
            return 1u;
        }
        s_nvm_pan_pending = 1u;
        s_nvm_dirty = 1u;
    }

    pan_stop_pulse();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_estop)
    {
        __set_PRIMASK(primask);
        return 1u;
    }
    s_pan_target_steps = target_steps;
    delta = target_steps - s_pan_current_steps;
    if (delta == 0)
    {
        pan_enable(1u);
        __set_PRIMASK(primask);
        return 0u;
    }

    s_pan_direction = delta > 0 ? 1u : 0u;
    if (s_pan_direction != 0u)
    {
        GPIO_SetBits(S31_PAN_DIR_PORT, S31_PAN_DIR_PIN);
        steps = (uint32_t)delta;
    }
    else
    {
        GPIO_ResetBits(S31_PAN_DIR_PORT, S31_PAN_DIR_PIN);
        steps = (uint32_t)(-delta);
    }

    period = (uint16_t)(1000000u * 60u /
                        (S31_PAN_MOVE_RPM * S31_PAN_STEPS_PER_REV));
    TIM_SetAutoreload(TIM13, (uint16_t)(period - 1u));
    TIM_SetCompare1(TIM13, (uint16_t)(period / 2u));
    TIM_GenerateEvent(TIM13, TIM_EventSource_Update);
    TIM_SetCounter(TIM13, 0u);
    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);

    s_pan_steps_remaining = steps;
    s_pan_moving = 1u;
    pan_enable(1u);
    TIM_ITConfig(TIM13, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM13, ENABLE);
    __set_PRIMASK(primask);
    return 0u;
}

/* ------------------------------------------------------------------------ */
/* v3.10.0 parameter records and flash persistence                          */

static void nvm_mark_dirty(void)
{
    s_nvm_dirty = 1u;
    s_nvm_idle_ticks = 0u;
}

static int32_t param_get(u8 id)
{
    return s_param_values[id - 1u];
}

static void param_apply(u8 id)
{
    switch (id)
    {
        case 1u:
            s_line_speed_percent = (uint8_t)s_param_values[0];
            break;
        case 2u:
            line_fusion_set_turn_gain((uint8_t)s_param_values[1]);
            break;
        case 3u:
            Velocity_KP = (float)s_param_values[2];
            break;
        case 4u:
            Velocity_KI = (float)s_param_values[3];
            break;
        case 5u:
            s_max_speed_mps = (float)s_param_values[4] / 1000.0f;
            line_fusion_set_max_speed_mm_s((uint16_t)s_param_values[4]);
            break;
        default:
            break;
    }
}

static u8 param_set(u8 id, int32_t value)
{
    if (pan_param_valid(id, value) == 0u)
    {
        return 1u;
    }
    s_param_values[id - 1u] = value;
    param_apply(id);
    nvm_mark_dirty();
    return 0u;
}

static void nvm_fill_record(C30D_NvmRecord *record, u16 flags)
{
    record->magic = C30D_NVM_MAGIC;
    record->version = C30D_NVM_VERSION;
    record->size = (u16)sizeof(C30D_NvmRecord);
    record->pan_steps = s_pan_current_steps;
    record->line_base_percent = (u16)s_param_values[0];
    record->line_turn_gain = (u16)s_param_values[1];
    record->speed_kp = (u16)s_param_values[2];
    record->speed_ki = (u16)s_param_values[3];
    record->max_speed_mms = (u16)s_param_values[4];
    record->reserved = flags;
    c30d_nvm_set_crc(record);
}

static bool nvm_slot_erased(const C30D_NvmRecord *record)
{
    const u8 *bytes = (const u8 *)record;
    for (unsigned i = 0; i < sizeof(*record); ++i)
    {
        if (bytes[i] != 0xFFu)
        {
            return false;
        }
    }
    return true;
}

static u8 nvm_append(u16 flags, u8 slots)
{
    C30D_NvmRecord record;
    C30D_NvmRecord check;
    u32 address;

    if (s_nvm_next + slots * sizeof(record) > C30D_NVM_END)
    {
        /* Erase stalls Flash fetches. Only reclaim while all motion is idle. */
        if (s_pan_moving || s_estop || s_test_active || s_line_enabled ||
            s_left_percent || s_right_percent || s_bt_left_percent ||
            s_bt_right_percent || s_active_source != 0u ||
            STMFLASH_EraseSector(STMFLASH_NVM_ADDR) != 0u)
        {
            return 1u;
        }
        s_nvm_next = STMFLASH_NVM_ADDR;
    }
    nvm_fill_record(&record, flags);
    address = s_nvm_next;
    u8 result = STMFLASH_Program(address, (const u8 *)&record, sizeof(record));
    STMFLASH_Read(address, (u8 *)&check, sizeof(check));
    if (!nvm_slot_erased(&check))
    {
        s_nvm_next += sizeof(record);
    }
    return result != 0u || memcmp(&record, &check, sizeof(record)) != 0 ? 1u : 0u;
}

static void nvm_load(void)
{
    C30D_NvmRecord record;
    C30D_NvmRecord last;
    u8 have_valid = 0u;
    u8 last_valid = 0u;

    s_nvm_next = STMFLASH_NVM_ADDR;
    while (s_nvm_next + sizeof(record) <= C30D_NVM_END)
    {
        STMFLASH_Read(s_nvm_next, (u8 *)&record, sizeof(record));
        if (nvm_slot_erased(&record))
        {
            break;
        }
        last_valid = c30d_nvm_valid(&record);
        if (last_valid != 0u)
        {
            last = record;
            have_valid = 1u;
        }
        s_nvm_next += sizeof(record);
    }
    if (have_valid != 0u)
    {
        (void)param_set(PAN_PARAM_LINE_BASE_PERCENT, last.line_base_percent);
        (void)param_set(PAN_PARAM_LINE_TURN_GAIN, last.line_turn_gain);
        (void)param_set(PAN_PARAM_SPEED_KP, last.speed_kp);
        (void)param_set(PAN_PARAM_SPEED_KI, last.speed_ki);
        (void)param_set(PAN_PARAM_MAX_SPEED_MMS, last.max_speed_mms);
    }
    s_nvm_dirty = 0u;
    s_nvm_idle_ticks = 0u;
    if (have_valid == 0u || last_valid == 0u ||
        !c30d_nvm_pan_known(&last, S31_PAN_STEPS_PER_180_DEG))
    {
        return;
    }
    s_pan_current_steps = last.pan_steps;
    s_pan_target_steps = last.pan_steps;
    s_pan_deg = pan_param_deg_from_steps(last.pan_steps, S31_PAN_STEPS_PER_180_DEG);
    s_fault_flags &= (u8)~S31_FAULT_GIMBAL_UNHOMED;
    s_pan_boot_pending = last.pan_steps != 0;
}

static void nvm_poll(void)
{
    u16 flags;

    if (s_nvm_dirty == 0u || s_pan_moving || Voltage < 10.0f)
    {
        return;
    }
    /* A finished move gets committed on the next control tick. Parameter
     * updates still coalesce for one second while the chassis is idle. */
    if (s_nvm_pan_pending == 0u)
    {
        if (s_estop || s_line_enabled || s_active_source != 0u ||
            s_left_percent || s_right_percent || s_bt_left_percent ||
            s_bt_right_percent)
        {
            s_nvm_idle_ticks = 0u;
            return;
        }
        if (s_nvm_idle_ticks < C30D_NVM_SAVE_DELAY_TICKS)
        {
            ++s_nvm_idle_ticks;
            return;
        }
    }
    flags = (s_fault_flags & S31_FAULT_GIMBAL_UNHOMED) != 0u ?
            C30D_NVM_PAN_MOVING : 0u;
    if (nvm_append(flags, 1u) == 0u)
    {
        s_nvm_dirty = 0u;
        s_nvm_pan_pending = 0u;
    }
    else
    {
        s_nvm_pan_pending = 0u;
        s_nvm_idle_ticks = 0u;
    }
}

static void put_u16_be(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

static void put_i16_be(uint8_t *out, int16_t value)
{
    put_u16_be(out, (uint16_t)value);
}

/* v3.10.0 parameter frames use big-endian int32 fields. */
static int32_t get_i32_be(const uint8_t *in)
{
    return (int32_t)(((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16) |
                     ((uint32_t)in[2] << 8) | (uint32_t)in[3]);
}

static void put_i32_be(uint8_t *out, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    out[0] = (uint8_t)(raw >> 24);
    out[1] = (uint8_t)(raw >> 16);
    out[2] = (uint8_t)(raw >> 8);
    out[3] = (uint8_t)raw;
}

static void chassis_set_pwm(int motor_a, int motor_b)
{
    if (motor_a < 0)
    {
        PWMA2 = 16799;
        PWMA1 = 16799 + motor_a;
    }
    else
    {
        PWMA1 = 16799;
        PWMA2 = 16799 - motor_a;
    }

    if (motor_b < 0)
    {
        PWMB1 = 16799;
        PWMB2 = 16799 + motor_b;
    }
    else
    {
        PWMB2 = 16799;
        PWMB1 = 16799 - motor_b;
    }

    PWMC1 = 16799;
    PWMC2 = 16799;
    PWMD1 = 16799;
    PWMD2 = 16799;
}

static void chassis_get_velocity(void)
{
    int raw_a = Read_Encoder(2);
    int raw_b = -Read_Encoder(3);

    (void)Read_Encoder(4);
    (void)Read_Encoder(5);

    s_raw_encoder_a = raw_a;
    s_raw_encoder_b = raw_b;
    MOTOR_A.Encoder =
        (float)raw_a * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_B.Encoder =
        (float)raw_b * CONTROL_FREQUENCY * Wheel_perimeter / Encoder_precision;
    MOTOR_C.Encoder = 0.0f;
    MOTOR_D.Encoder = 0.0f;
}

static uint8_t chassis_turn_off(float voltage)
{
    if (voltage < 10.0f)
    {
        s_fault_flags |= S31_FAULT_LOW_BATTERY;
    }
    else
    {
        s_fault_flags &= (uint8_t)~S31_FAULT_LOW_BATTERY;
    }
    if (voltage < 10.0f || EN == 0 || Flag_Stop != 0u)
    {
        chassis_set_pwm(0, 0);
        return 1u;
    }
    return 0u;
}

static void battery_adc_init(void)
{
    ADC_CommonInitTypeDef common;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    memset(&common, 0, sizeof(common));
    common.ADC_Mode = ADC_Mode_Independent;
    common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    common.ADC_Prescaler = ADC_Prescaler_Div6;
    ADC_CommonInit(&common);

    memset(&adc, 0, sizeof(adc));
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);
    delay_ms(1);
}

static void board_init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(168u);
    I2C_GPIOInit();
    SYS_VAL_t_Init(&SysVal);

    if (MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID())
    {
        SysVal.HardWare_Ver = V1_0;
    }
    else if (REG_VAL_WIA == ICM20948_getDeviceID())
    {
        SysVal.HardWare_Ver = V1_1;
    }
    else
    {
        NVIC_SystemReset();
    }

    Enable_Pin();
    Adc_Init();
    Adc_POWER_Init();
    battery_adc_init();

    TIM1_PWM_Init(16799u, 0u);
    TIM9_PWM_Init(16799u, 0u);
    TIM10_PWM_Init(16799u, 0u);
    TIM11_PWM_Init(16799u, 0u);

    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();
    Encoder_Init_TIM5();
    pan_gpio_timer_init();

    Car_Mode = Tank_Car;
    Robot_Init(S31_TRACK_WHEEL_SPACING_M, 0.0f, 0.0f,
               S31_TRACK_GEAR_RATIO, S31_TRACK_ENCODER_PPR,
               S31_TRACK_WHEEL_DIAMETER_M);
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[S31_MAX_FRAME];
    uint8_t frame_size = s31_frame_encode(cmd, payload, length, frame);
    uint32_t wait;

    if (frame_size == 0u)
    {
        return;
    }

    for (uint8_t i = 0u; i < frame_size; ++i)
    {
        wait = 100000u;
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        {
            if (--wait == 0u)
            {
                ++s_uart3_txe_timeouts;
                return;
            }
        }
        USART_SendData(USART3, frame[i]);
    }
    wait = 100000u;
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
        if (--wait == 0u)
        {
            ++s_uart3_tc_timeouts;
            return;
        }
    }
}

static void reset_pi(S31_PI_t *pi)
{
    pi->pwm = 0.0f;
    pi->last_bias = 0.0f;
}

static float pi_step(S31_PI_t *pi, float actual, float target)
{
    float bias = target - actual;

    pi->pwm += Velocity_KP * (bias - pi->last_bias) + Velocity_KI * bias;
    pi->last_bias = bias;

    if (pi->pwm > 16500.0f)
    {
        pi->pwm = 16500.0f;
    }
    else if (pi->pwm < -16500.0f)
    {
        pi->pwm = -16500.0f;
    }
    return pi->pwm;
}

static void halt_motor_output(void)
{
    reset_pi(&s_pi_a);
    reset_pi(&s_pi_b);
    MOTOR_A.Target = 0.0f;
    MOTOR_B.Target = 0.0f;
    MOTOR_C.Target = 0.0f;
    MOTOR_D.Target = 0.0f;
    MOTOR_A.Motor_Pwm = 0.0f;
    MOTOR_B.Motor_Pwm = 0.0f;
    MOTOR_C.Motor_Pwm = 0.0f;
    MOTOR_D.Motor_Pwm = 0.0f;
    chassis_set_pwm(0, 0);
}

static void send_command_ack(uint8_t command, uint8_t status)
{
    uint8_t payload[2];

    payload[0] = command;
    payload[1] = status;
    send_frame(S31_CMD_COMMAND_ACK, payload, sizeof(payload));
}

/* 0x33 PARAM_STATE: id u8 + value i32 big-endian (same layout as SET_PARAM). */
static void send_param_state(uint8_t id, int32_t value)
{
    uint8_t payload[5];

    payload[0] = id;
    put_i32_be(&payload[1], value);
    send_frame(S31_CMD_PARAM_STATE, payload, sizeof(payload));
}

static bool bt_stop_holdoff_active(uint32_t now)
{
    return s_bt_stop_holdoff_tick != 0u &&
           (uint32_t)(now - s_bt_stop_holdoff_tick) < 1000u;
}

static void stop_chassis_now(void)
{
    s_test_active = 0u;
    s_test_pwm_a = 0;
    s_test_pwm_b = 0;
    s_bt_armed = 0u;
    s_bt_left_percent = 0;
    s_bt_right_percent = 0;
    s_bt_direction_action = TD_BT_ACTION_NONE;
    halt_motor_output();
}

static void stop_motors_now(void)
{
    pan_stop_pulse();
    stop_chassis_now();
}

static void disable_line(void)
{
    s_line_enabled = 0u;
    s_line_bt_mode = 0u;
    s_line_left_percent = 0;
    s_line_right_percent = 0;
}

static void release_bt_drive(void)
{
    s_bt_armed = 0u;
    s_bt_left_percent = 0;
    s_bt_right_percent = 0;
    s_bt_direction_action = TD_BT_ACTION_NONE;
}

static u8 apply_drive(const uint8_t *payload, uint8_t length, uint32_t now, bool holdoff)
{
    if (length != 2u)
    {
        return 1u;
    }
    int8_t left = (int8_t)payload[0];
    int8_t right = (int8_t)payload[1];
    if (left < -100 || left > 100 || right < -100 || right > 100 || s_estop)
    {
        return 1u;
    }
    bool moving = left != 0 || right != 0;
    if (moving && holdoff)
    {
        s_bt_stop_holdoff_tick = now;
        return 1u;
    }
    if (moving && (EN == 0 || Flag_Stop != 0u || Voltage < 10.0f ||
        line_motion_ultrasonic_guard(s_ultrasonic_ready && s_ultrasonic_cm != 0u,
                                     s_ultrasonic_cm, left, right) == LINE_GUARD_STOP))
    {
        return 1u;
    }

    disable_line();
    s_left_percent = left;
    s_right_percent = right;
    s_last_drive_tick = now;
    s_armed = 1u;
    s_fault_flags &= (uint8_t)~S31_FAULT_CMD_TIMEOUT;
    ++s_drive_frame_count;
    if (moving)
    {
        s_motion_stop_latched = 0u;
        ++s_nonzero_drive_count;
        s_last_nonzero_drive_tick = now;
        release_bt_drive();
    }
    return 0u;
}

static u8 apply_line(const uint8_t *payload, uint8_t length, uint32_t now, bool holdoff)
{
    if ((length != 2u && length != 3u) ||
        payload[0] > 1u || payload[1] < 5u || payload[1] > 100u)
    {
        return 1u;
    }
    /* 旧版两字节帧的偏置为零。扩展帧仅用于低速巡线。 */
    int8_t bias = length == 3u ? (int8_t)payload[2] : 0;
    if (length == 3u && (payload[0] == 0u || payload[1] > 15u || bias < -5 || bias > 5))
    {
        return 1u;
    }
    if (payload[0] == 0u)
    {
        s_motion_stop_latched = 1u;
        disable_line();
        return 0u;
    }
    if (holdoff)
    {
        s_bt_stop_holdoff_tick = now;
        return 1u;
    }
    if (s_estop || EN == 0 || Flag_Stop != 0u || Voltage < 10.0f)
    {
        return 1u;
    }
    if (!s_line_enabled)
    {
        line_motion_init(&s_line_motion);
    }
    s_motion_stop_latched = 0u;
    s_line_enabled = 1u;
    s_line_bt_mode = 0u;
    s_line_speed_percent = payload[1];
    s_line_vision_bias = bias;
    s_line_last_cmd_tick = now;
    s_last_drive_tick = now;
    s_armed = 1u;
    s_fault_flags &= (uint8_t)~S31_FAULT_CMD_TIMEOUT;
    release_bt_drive();
    return 0u;
}

static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t length)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    bool holdoff = bt_stop_holdoff_active(now);

    switch (cmd)
    {
        case S31_CMD_DRIVE:
            send_command_ack(cmd, apply_drive(payload, length, now, holdoff));
            break;

        case S31_CMD_LINE:
            send_command_ack(cmd, apply_line(payload, length, now, holdoff));
            break;

        case S31_CMD_STOP_ALL:
            if (length != 0u)
            {
                send_command_ack(cmd, 1u);
                break;
            }
            s_motion_stop_latched = 1u;
            disable_line();
            s_left_percent = 0;
            s_right_percent = 0;
            s_armed = 0u;
            stop_chassis_now();
            send_command_ack(cmd, 0u);
            break;

        case S31_CMD_ACTUATOR:
            if (length == 3u && payload[2] <= 2u && !s_pan_boot_pending &&
                !s_pan_boot_moving)
            {
                uint16_t deg = ((uint16_t)payload[0] << 8) | payload[1];
                send_command_ack(cmd, deg <= PAN_MAX_DEG ?
                                 pan_move_to_steps(pan_deg_to_steps(deg)) : 1u);
            }
            else
            {
                send_command_ack(cmd, 1u);
            }
            break;

        case S31_CMD_CALIBRATE_PAN:
            /* Human confirms the camera is already facing the physical home. */
            if (length != 0u || s_estop || s_pan_moving || s_active_source ||
                s_line_enabled || s_left_percent || s_right_percent ||
                s_bt_left_percent || s_bt_right_percent || Voltage < 10.0f)
            {
                send_command_ack(cmd, 1u);
                break;
            }
            pan_stop_pulse();
            s_pan_boot_pending = 0u;
            s_pan_current_steps = 0;
            s_pan_target_steps = 0;
            s_pan_deg = PAN_HOME_DEG;
            if (nvm_append(0u, 1u) != 0u)
            {
                s_fault_flags |= S31_FAULT_GIMBAL_UNHOMED;
                send_command_ack(cmd, 1u);
                break;
            }
            s_nvm_dirty = 0u;
            s_nvm_pan_pending = 0u;
            s_fault_flags &= (uint8_t)~S31_FAULT_GIMBAL_UNHOMED;
            send_command_ack(cmd, 0u);
            break;

        case S31_CMD_SET_PARAM:
            if (length == C30D_PARAM_FRAME_LEN)
            {
                uint8_t status =
                    param_set(payload[0], get_i32_be(&payload[1]));

                send_command_ack(cmd, status);
                if (status == 0u)
                {
                    /* Mirror the accepted value back to the S31 host. */
                    send_param_state(payload[0], param_get(payload[0]));
                }
            }
            else
            {
                send_command_ack(cmd, 1u);
            }
            break;

        case S31_CMD_GET_PARAM:
            if (length >= 1u && payload[0] >= 1u &&
                payload[0] <= C30D_PARAM_COUNT)
            {
                send_param_state(payload[0], param_get(payload[0]));
            }
            else
            {
                send_command_ack(cmd, 1u);
            }
            break;

        case S31_CMD_ESTOP:
            s_estop = 1u;
            s_line_enabled = 0u;
            s_line_bt_mode = 0u;
            s_line_left_percent = 0;
            s_line_right_percent = 0;
            s_fault_flags |= S31_FAULT_ESTOP;
            s_bt_armed = 0u;
            s_bt_left_percent = 0;
            s_bt_right_percent = 0;
            s_bt_direction_action = TD_BT_ACTION_NONE;
            s_test_active = 0u;
            s_test_pwm_a = 0;
            s_test_pwm_b = 0;
            stop_motors_now();
            send_command_ack(cmd, 0u);
            break;

        case S31_CMD_HEARTBEAT:
            /* Heartbeat proves USB is alive but never extends drive timeout. */
            send_command_ack(cmd, 0u);
            break;

        case S31_CMD_CLEAR_ESTOP:
            if (length != 0u)
            {
                send_command_ack(cmd, 1u);
                break;
            }
            s_estop = 0u;
            s_motion_stop_latched = 1u;
            s_left_percent = 0;
            s_right_percent = 0;
            s_last_drive_tick = (uint32_t)now;
            s_fault_flags &= (uint8_t)~S31_FAULT_ESTOP;
            s_bt_armed = 0u;
            s_bt_left_percent = 0;
            s_bt_right_percent = 0;
            s_bt_direction_action = TD_BT_ACTION_NONE;
            s_test_active = 0u;
            s_test_pwm_a = 0;
            s_test_pwm_b = 0;
            stop_motors_now();
            send_command_ack(cmd, 0u);
            break;

        default:
            break;
    }
}

/* The controller task dispatches frames. The USART ISR fills the ring and
 * recognizes the fixed emergency-stop frame on its priority path. */
static void poll_rx(void)
{
    uint8_t byte;
    uint16_t next;

    while (s_rx_tail != s_rx_head)
    {
        byte = s_rx_ring[s_rx_tail];
        next = (uint16_t)((s_rx_tail + 1u) & (S31_RX_RING_SIZE - 1u));
        s_rx_tail = next;
        s31_frame_result_t result = s31_frame_feed(&s_parser, byte);
        if (result == S31_FRAME_READY)
        {
            handle_frame(s_parser.command, s_parser.payload, s_parser.length);
        }
        else if (result == S31_FRAME_BAD_CRC)
        {
            ++s_crc_error_count;
        }
    }
}

void USART3_IRQHandler(void)
{
    uint8_t byte;
    uint16_t next;
    uint32_t previous;

    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        byte = (uint8_t)USART3->DR;
        ++s_rx_overflow_count;
    }
    else if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        byte = (uint8_t)USART_ReceiveData(USART3);
    }
    else
    {
        return;
    }

    /*
     * AA 55 00 12 7E is the fixed emergency-stop frame.  Checking its last
     * five bytes in the ISR removes motor torque before the scheduler runs.
     */
    previous = s_estop_history;
    s_estop_history = (s_estop_history << 8) | byte;
    if (byte == 0x7Eu && previous == 0xAA550012UL)
    {
        s_estop = 1u;
        s_fault_flags |= S31_FAULT_ESTOP;
        s_pan_boot_pending = 0u;
        pan_stop_pulse();
        chassis_set_pwm(0, 0);
    }

    next = (uint16_t)((s_rx_head + 1u) & (S31_RX_RING_SIZE - 1u));
    if (next != s_rx_tail)
    {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = next;
    }
    else
    {
        ++s_rx_overflow_count;
    }
}

void TIM8_UP_TIM13_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM13, TIM_IT_Update) == RESET)
    {
        return;
    }

    TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
    if (s_pan_moving == 0u)
    {
        return;
    }

    pan_count_step();
    if (s_pan_steps_remaining == 0u)
    {
        /* Hold torque after the last complete pulse. */
        pan_finish_pulse();
    }
}

static void ultrasonic_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = S31_ULTRA_TRIG_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(S31_ULTRA_TRIG_PORT, &gpio);

    gpio.GPIO_Pin = S31_ULTRA_ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(S31_ULTRA_ECHO_PORT, &gpio);

    GPIO_ResetBits(S31_ULTRA_TRIG_PORT, S31_ULTRA_TRIG_PIN);
}

static uint32_t wait_cycles(uint32_t start, uint32_t timeout_cycles)
{
    return (uint32_t)(DWT->CYCCNT - start);
}

static uint16_t ultrasonic_read_cm(void)
{
    uint32_t start;
    uint32_t pulse_cycles;
    uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    uint16_t distance_cm;

    if (cycles_per_us == 0u)
    {
        return 0u;
    }

    GPIO_ResetBits(S31_ULTRA_TRIG_PORT, S31_ULTRA_TRIG_PIN);
    delay_us(2u);
    GPIO_SetBits(S31_ULTRA_TRIG_PORT, S31_ULTRA_TRIG_PIN);
    delay_us(10u);
    GPIO_ResetBits(S31_ULTRA_TRIG_PORT, S31_ULTRA_TRIG_PIN);

    start = DWT->CYCCNT;
    while (GPIO_ReadInputDataBit(S31_ULTRA_ECHO_PORT,
                                 S31_ULTRA_ECHO_PIN) == RESET)
    {
        if (wait_cycles(start, cycles_per_us * 5000u) >
            cycles_per_us * 5000u)
        {
            return 0u;
        }
    }

    start = DWT->CYCCNT;
    while (GPIO_ReadInputDataBit(S31_ULTRA_ECHO_PORT,
                                 S31_ULTRA_ECHO_PIN) != RESET)
    {
        if (wait_cycles(start, cycles_per_us * 30000u) >
            cycles_per_us * 30000u)
        {
            return 0u;
        }
    }

    pulse_cycles = DWT->CYCCNT - start;
    distance_cm = (uint16_t)((pulse_cycles / cycles_per_us + 29u) / 58u);
    if (distance_cm < 2u || distance_cm > 400u)
    {
        return 0u;
    }
    return distance_cm;
}

static uint16_t ultrasonic_read_filtered(void)
{
    uint16_t value = ultrasonic_read_cm();
    uint16_t a;
    uint16_t b;
    uint16_t c;
    uint16_t temporary;

    s_ultrasonic_history[s_ultrasonic_history_index] = value;
    s_ultrasonic_history_index =
        (uint8_t)((s_ultrasonic_history_index + 1u) % 3u);
    if (s_ultrasonic_history_count < 3u)
    {
        ++s_ultrasonic_history_count;
    }

    if (s_ultrasonic_history_count < 3u)
    {
        return value;
    }

    a = s_ultrasonic_history[0];
    b = s_ultrasonic_history[1];
    c = s_ultrasonic_history[2];
    if (a > b)
    {
        temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c)
    {
        temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b)
    {
        temporary = a;
        a = b;
        b = temporary;
    }
    return b;
}

static void line_sensor_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOC, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;

    gpio.GPIO_Pin = S31_LINE_DH1_PIN | S31_LINE_DH2_PIN;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = S31_LINE_DH3_PIN | S31_LINE_DH4_PIN;
    GPIO_Init(GPIOC, &gpio);
}

static uint8_t line_sensor_read(void)
{
    uint8_t bits = 0u;

    if (GPIO_ReadInputDataBit(S31_LINE_DH1_PORT,
                              S31_LINE_DH1_PIN) != RESET)
    {
        bits |= 0x08u;
    }
    if (GPIO_ReadInputDataBit(S31_LINE_DH2_PORT,
                              S31_LINE_DH2_PIN) != RESET)
    {
        bits |= 0x04u;
    }
    if (GPIO_ReadInputDataBit(S31_LINE_DH3_PORT,
                              S31_LINE_DH3_PIN) != RESET)
    {
        bits |= 0x02u;
    }
    if (GPIO_ReadInputDataBit(S31_LINE_DH4_PORT,
                              S31_LINE_DH4_PIN) != RESET)
    {
        bits |= 0x01u;
    }
    return bits;
}

static void line_sensor_update(uint32_t now)
{
    line_motion_input_t input;
    line_motion_output_t output;
    uint8_t bits = line_sensor_read();

    s_line_bits = bits;
    if (s_ccd_last_tick == 0u ||
        (uint32_t)(now - s_ccd_last_tick) >=
            pdMS_TO_TICKS(40u))
    {
        s_ccd_last_tick = now;
        s_ccd_valid =
            tsl1401_read(s_ccd_pixels, &s_ccd_elapsed_us) ? 1u : 0u;
        (void)line_fusion_analyze_ccd(s_ccd_pixels, &s_line_ccd);
    }

    memset(&input, 0, sizeof(input));
    input.enabled = s_line_enabled != 0u;
    input.ccd_valid = s_ccd_valid != 0u;
    input.ccd = s_line_ccd;
    input.lf_bits = bits;
    input.obstacle_valid =
        s_ultrasonic_ready != 0u && s_ultrasonic_cm != 0u;
    input.distance_cm = s_ultrasonic_cm;
    input.left_speed_mm_s =
        clamp_i16((int32_t)(MOTOR_A.Encoder * 1000.0f));
    input.right_speed_mm_s =
        clamp_i16((int32_t)(MOTOR_B.Encoder * 1000.0f));
    input.speed_percent = s_line_speed_percent;
    if (!s_line_bt_mode &&
        (uint32_t)(now - s_line_last_cmd_tick) <= pdMS_TO_TICKS(250u))
    {
        input.vision_bias_percent = s_line_vision_bias;
    }
    input.elapsed_ms = 10u;

    line_motion_step(&s_line_motion, &input, &output);
    if (input.enabled)
    {
        s_line_left_percent = output.left_percent;
        s_line_right_percent = output.right_percent;
    }
    else
    {
        s_line_left_percent = 0;
        s_line_right_percent = 0;
    }
    s_line_source = output.source;
    s_line_error = output.line_error;
    s_line_phase = output.phase;
    s_line_rewind_progress = output.rewind_progress;
}

static int16_t clamp_i16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static void update_yaw(TickType_t now)
{
    float raw_z;
    float delta_cdeg;
    uint32_t elapsed_ms = (uint32_t)(now - s_last_yaw_tick);

    s_last_yaw_tick = now;
    if (elapsed_ms > 100u)
    {
        elapsed_ms = 100u;
    }

    if (!s_imu_ready)
    {
        s_fault_flags |= S31_FAULT_IMU_OFFLINE;
        return;
    }

    if (SysVal.HardWare_Ver == V1_1)
    {
        (void)ICM20948_Get_Gyroscope();
        (void)ICM20948_Get_Accel();
        raw_z = (float)imu.gyro.z;
    }
    else
    {
        MPU6050_Get_Gyroscope();
        MPU6050_Get_Accelscope();
        raw_z = (float)imu.gyro.z;
    }

    s_imu_gyro_ddps[0] =
        clamp_i16((int32_t)((float)imu.gyro.x * 10.0f / 65.5f));
    s_imu_gyro_ddps[1] =
        clamp_i16((int32_t)((float)imu.gyro.y * 10.0f / 65.5f));
    s_imu_gyro_ddps[2] =
        clamp_i16((int32_t)((float)imu.gyro.z * 10.0f / 65.5f));
    s_imu_accel_mg[0] =
        clamp_i16((int32_t)((float)imu.accel.x * 1000.0f / 16384.0f));
    s_imu_accel_mg[1] =
        clamp_i16((int32_t)((float)imu.accel.y * 1000.0f / 16384.0f));
    s_imu_accel_mg[2] =
        clamp_i16((int32_t)((float)imu.accel.z * 1000.0f / 16384.0f));

    /* Both supported IMUs are configured for +/-500 dps, 65.5 LSB/dps. */
    delta_cdeg = raw_z * (float)elapsed_ms / (10.0f * 65.5f);
    s_yaw_cdeg = (int16_t)(s_yaw_cdeg + (int16_t)delta_cdeg);
    if (s_yaw_cdeg > 18000)
    {
        s_yaw_cdeg = (int16_t)(s_yaw_cdeg - 36000);
    }
    else if (s_yaw_cdeg < -18000)
    {
        s_yaw_cdeg = (int16_t)(s_yaw_cdeg + 36000);
    }
}

static void send_chassis_state(void)
{
    uint8_t payload[6];

    put_i16_be(&payload[0], clamp_i16((int32_t)(MOTOR_A.Encoder * 1000.0f)));
    put_i16_be(&payload[2], clamp_i16((int32_t)(MOTOR_B.Encoder * 1000.0f)));
    put_i16_be(&payload[4], s_yaw_cdeg);
    send_frame(S31_CMD_CHASSIS_STATE, payload, sizeof(payload));
}

static void send_sensor_state(void)
{
    uint8_t payload[6];
    uint16_t battery_mv = 0u;

    if (Voltage > 0.0f && Voltage < 30.0f)
    {
        battery_mv = (uint16_t)(Voltage * 1000.0f);
    }

    put_u16_be(&payload[0], s_ultrasonic_cm);
    payload[2] = s_line_bits;
    payload[3] = (uint8_t)(
        0x01u |
        ((s_active_source & 0x07u) << 1u) |
        (s_line_enabled ? 0x80u : 0u));
    put_u16_be(&payload[4], battery_mv);
    send_frame(S31_CMD_SENSOR_STATE, payload, sizeof(payload));
}

static void send_line_state(void)
{
    uint8_t payload[10];
    uint8_t flags = 0u;
    uint8_t distance = s_ultrasonic_cm > 254u ?
        254u : (uint8_t)s_ultrasonic_cm;

    if (s_line_enabled)
    {
        flags |= S31_LINE_FLAG_ACTIVE;
    }
    if (s_line_ccd.found)
    {
        flags |= S31_LINE_FLAG_CCD_FOUND;
    }
    if (s_ccd_valid)
    {
        flags |= S31_LINE_FLAG_CCD_VALID;
    }
    if (s_line_phase == LINE_PHASE_BRAKE ||
        s_line_phase == LINE_PHASE_REWIND ||
        s_line_phase == LINE_PHASE_WAIT_CLEAR)
    {
        flags |= S31_LINE_FLAG_OBSTACLE;
    }
    if (s_line_phase == LINE_PHASE_REWIND)
    {
        flags |= S31_LINE_FLAG_REWIND;
    }
    if (s_line_phase == LINE_PHASE_WAIT_CLEAR)
    {
        flags |= S31_LINE_FLAG_WAIT_CLEAR;
    }

    payload[0] = flags;
    payload[1] = s_line_source;
    payload[2] = s_line_ccd.center;
    payload[3] = s_line_ccd.confidence;
    payload[4] = (uint8_t)(int8_t)(
        s_line_error < -100 ? -100 :
        s_line_error > 100 ? 100 : s_line_error);
    payload[5] = s_line_bits;
    payload[6] = (uint8_t)s_active_left_percent;
    payload[7] = (uint8_t)s_active_right_percent;
    payload[8] = s_line_rewind_progress;
    payload[9] = distance;
    send_frame(S31_CMD_LINE_STATE, payload, sizeof(payload));
}

static void send_imu_state(void)
{
    uint8_t payload[12];

    put_i16_be(&payload[0], s_imu_gyro_ddps[0]);
    put_i16_be(&payload[2], s_imu_gyro_ddps[1]);
    put_i16_be(&payload[4], s_imu_gyro_ddps[2]);
    put_i16_be(&payload[6], s_imu_accel_mg[0]);
    put_i16_be(&payload[8], s_imu_accel_mg[1]);
    put_i16_be(&payload[10], s_imu_accel_mg[2]);
    send_frame(S31_CMD_IMU_STATE, payload, sizeof(payload));
}

static void send_motor_state(void)
{
    uint8_t payload[18];

    put_i16_be(&payload[0], clamp_i16(s_raw_encoder_a));
    put_i16_be(&payload[2], clamp_i16(s_raw_encoder_b));
    put_i16_be(&payload[4], clamp_i16((int32_t)MOTOR_A.Motor_Pwm));
    put_i16_be(&payload[6], clamp_i16((int32_t)MOTOR_B.Motor_Pwm));
    put_i16_be(&payload[8],
               clamp_i16((int32_t)(MOTOR_A.Target * 1000.0f)));
    put_i16_be(&payload[10],
               clamp_i16((int32_t)(MOTOR_B.Target * 1000.0f)));
    put_u16_be(&payload[12], s_pan_deg);
    payload[14] = s_active_source;
    payload[15] = (uint8_t)s_active_left_percent;
    payload[16] = (uint8_t)s_active_right_percent;
    payload[17] = ((s_fault_flags & S31_FAULT_GIMBAL_UNHOMED) == 0u ? 1u : 0u) |
                  (s_pan_moving != 0u ? 2u : 0u) |
                  (s_pan_boot_pending || s_pan_boot_moving ? 4u : 0u);
    send_frame(S31_CMD_MOTOR_STATE, payload, sizeof(payload));
}

static void send_safety_state(TickType_t now)
{
    uint8_t payload[8];
    uint32_t age;
    uint32_t uptime_s;
    uint8_t state;

    if (s_armed)
    {
        age = (uint32_t)(now - s_last_drive_tick);
    }
    else
    {
        age = 0xFFFFu;
    }
    if (age > 0xFFFFu)
    {
        age = 0xFFFFu;
    }

    if (s_estop)
    {
        state = S31_ROBOT_ESTOP;
    }
    else if (s_armed)
    {
        state = S31_ROBOT_ACTIVE;
    }
    else
    {
        state = S31_ROBOT_IDLE;
    }

    uptime_s = (uint32_t)now / 1000u;
    if (uptime_s > 0xFFFFu)
    {
        uptime_s = 0xFFFFu;
    }

    payload[0] = state;
    payload[1] = s_fault_flags;
    put_u16_be(&payload[2], (uint16_t)uptime_s);
    payload[4] = S31_FIRMWARE_MAJOR;
    payload[5] = S31_FIRMWARE_MINOR;
    put_u16_be(&payload[6], (uint16_t)age);
    send_frame(S31_CMD_SAFETY_STATE, payload, sizeof(payload));
}

static void debug_uart1_send_char(char value)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(USART1, (uint16_t)(uint8_t)value);
}

static void debug_uart1_send(const char *text)
{
    while (text != NULL && *text != '\0')
    {
        debug_uart1_send_char(*text++);
    }
}

static void debug_uart1_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate = S31_BAUDRATE;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

static float sample_battery_voltage(void)
{
    uint32_t wait = 100000u;
    uint16_t raw;

    ADC_RegularChannelConfig(ADC1, Battery_Ch, 1u,
                             ADC_SampleTime_480Cycles);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if (--wait == 0u)
        {
            ++s_battery_adc_timeouts;
            return 0.0f;
        }
    }
    raw = ADC_GetConversionValue(ADC1);
    return (float)raw * 3.3f * 11.0f * 0.99f / 4096.0f;
}

static void oled_show_signed_mps(uint8_t x, uint8_t y, float speed_mps)
{
    int32_t value = (int32_t)(speed_mps * 100.0f);

    if (value < 0)
    {
        OLED_ShowString(x, y, (const u8 *)"-");
        value = -value;
    }
    else
    {
        OLED_ShowString(x, y, (const u8 *)"+");
    }
    OLED_ShowNumber((uint8_t)(x + 8u), y, (u32)value, 3u, 12u);
}

static void oled_refresh_status(TickType_t now)
{
    uint16_t battery_mv = 0u;
    uint32_t drive_age;
    uint32_t nonzero_age;

    if (Voltage > 0.0f && Voltage < 30.0f)
    {
        battery_mv = (uint16_t)(Voltage * 1000.0f);
    }
    drive_age = s_armed ? (uint32_t)(now - s_last_drive_tick) : 0xFFFFFFFFu;
    nonzero_age = (s_nonzero_drive_count != 0u) ?
                  (uint32_t)(now - s_last_nonzero_drive_tick) :
                  0xFFFFFFFFu;

    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    OLED_ShowString(0u, 0u, (const u8 *)"GOUMANG R3 TRACK");

    if (s_estop)
    {
        OLED_ShowString(0u, 16u, (const u8 *)"ESTOP");
    }
    else if (s_test_active)
    {
        OLED_ShowString(0u, 16u, (const u8 *)"TEST OPENLOOP");
    }
    else if (EN == 0)
    {
        OLED_ShowString(0u, 16u, (const u8 *)"ENABLE OFF");
    }
    else if (Voltage < 10.0f)
    {
        OLED_ShowString(0u, 16u, (const u8 *)"LOW VOLT");
    }
    else if (s_armed && drive_age <= S31_DRIVE_TIMEOUT_MS &&
             nonzero_age <= S31_DRIVE_TIMEOUT_MS)
    {
        OLED_ShowString(0u, 16u, (const u8 *)"S31:CMD");
    }
    else if (s_armed && drive_age <= S31_DRIVE_TIMEOUT_MS)
    {
        OLED_ShowString(0u, 16u, (const u8 *)"S31:ONLINE");
    }
    else
    {
        OLED_ShowString(0u, 16u, (const u8 *)"S31:WAIT");
    }

    OLED_ShowString(0u, 32u, (const u8 *)"V:");
    OLED_ShowNumber(16u, 32u, (u32)(battery_mv / 1000u), 2u, 12u);
    OLED_ShowString(32u, 32u, (const u8 *)".");
    OLED_ShowNumber(40u, 32u, (u32)(battery_mv % 1000u), 3u, 12u);
    OLED_ShowString(64u, 32u, (const u8 *)"D:");
    if (s_ultrasonic_cm == 0u)
    {
        OLED_ShowString(80u, 32u, (const u8 *)"---");
    }
    else
    {
        OLED_ShowNumber(80u, 32u, (u32)s_ultrasonic_cm, 3u, 12u);
    }
    OLED_ShowString(104u, 32u, (const u8 *)"cm");

    OLED_ShowString(0u, 48u, (const u8 *)"L");
    oled_show_signed_mps(8u, 48u, MOTOR_A.Encoder);
    OLED_ShowString(48u, 48u, (const u8 *)"R");
    oled_show_signed_mps(56u, 48u, MOTOR_B.Encoder);
    OLED_ShowString(96u, 48u, (const u8 *)"m/s");
    OLED_Refresh_Gram();
}

static void oled_show_signed_decimal(uint8_t x, uint8_t y,
                                     int32_t value, uint8_t width)
{
    if (value < 0)
    {
        OLED_ShowChar(x, y, '-', 12u, 1u);
        value = -value;
    }
    else
    {
        OLED_ShowChar(x, y, '+', 12u, 1u);
    }
    OLED_ShowNumber((uint8_t)(x + 8u), y, (u32)value, width, 12u);
}

static void oled_refresh_drive_diagnostic(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    OLED_ShowString(0u, 0u,
                    (const u8 *)(s_test_active ? "GOUMANG TEST" :
                                                   "GOUMANG DRIVE"));

    OLED_ShowString(0u, 16u, (const u8 *)"C");
    oled_show_signed_decimal(8u, 16u, s_left_percent, 3u);
    oled_show_signed_decimal(48u, 16u, s_right_percent, 3u);

    OLED_ShowString(0u, 32u, (const u8 *)"P");
    oled_show_signed_decimal(8u, 32u,
                             (int32_t)MOTOR_A.Motor_Pwm, 5u);
    oled_show_signed_decimal(64u, 32u,
                             (int32_t)MOTOR_B.Motor_Pwm, 5u);

    OLED_ShowString(0u, 48u, (const u8 *)"R");
    oled_show_signed_decimal(8u, 48u, s_raw_encoder_a, 4u);
    oled_show_signed_decimal(64u, 48u, s_raw_encoder_b, 4u);
    OLED_Refresh_Gram();
}

static void uart3_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);

    uart.USART_BaudRate = S31_BAUDRATE;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &uart);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART3, ENABLE);
}

void s31_link_init(void)
{
    board_init();

    /*
     * The chassis is now a tracked vehicle.  Ignore the potentiometer mode
     * selector so a wrong knob position cannot change wheel signs or PID
     * scale on the new controller board.
     */
    Car_Mode = Tank_Car;
    Robot_Init(S31_TRACK_WHEEL_SPACING_M, 0.0f, 0.0f,
               S31_TRACK_GEAR_RATIO, S31_TRACK_ENCODER_PPR,
               S31_TRACK_WHEEL_DIAMETER_M);

    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_overflow_count = 0u;
    s_estop_history = 0u;
    s_estop = 0u;
    s_armed = 0u;
    s_motion_stop_latched = 0u;
    s_bt_stop_holdoff_tick = 0u;
    s_last_drive_tick = 0u;
    s_left_percent = 0;
    s_right_percent = 0;
    s_drive_frame_count = 0u;
    s_nonzero_drive_count = 0u;
    s_last_nonzero_drive_tick = 0u;
    s_bt_armed = 0u;
    s_bt_left_percent = 0;
    s_bt_right_percent = 0;
    s_bt_last_command_tick = 0u;
    s_bt_command_count = 0u;
    s_bt_direction_action = TD_BT_ACTION_NONE;
    s_bt_speed_percent = TD_BT_DEFAULT_SPEED_PERCENT;
    s_line_enabled = 0u;
    s_line_bt_mode = 0u;
    s_line_speed_percent = (uint8_t)s_param_values[0];
    s_line_last_cmd_tick = 0u;
    s_line_bits = 0x0Fu;
    s_line_left_percent = 0;
    s_line_right_percent = 0;
    memset(&s_line_ccd, 0, sizeof(s_line_ccd));
    memset(s_ccd_pixels, 0, sizeof(s_ccd_pixels));
    s_ccd_elapsed_us = 0u;
    s_ccd_last_tick = 0u;
    s_ccd_valid = 0u;
    s_line_source = LINE_SOURCE_NONE;
    s_line_error = 0;
    s_line_phase = LINE_PHASE_IDLE;
    s_line_rewind_progress = 0u;
    memset(s_ultrasonic_history, 0, sizeof(s_ultrasonic_history));
    s_ultrasonic_history_count = 0u;
    s_ultrasonic_history_index = 0u;
    memset(s_imu_gyro_ddps, 0, sizeof(s_imu_gyro_ddps));
    memset(s_imu_accel_mg, 0, sizeof(s_imu_accel_mg));
    s_active_left_percent = 0;
    s_active_right_percent = 0;
    s_active_source = 0u;
    line_motion_init(&s_line_motion);
    s_test_active = 0u;
    s_test_pwm_a = 0;
    s_test_pwm_b = 0;
    s_test_started_tick = 0u;
    s_test_code = '-';
    s_raw_encoder_a = 0;
    s_raw_encoder_b = 0;
    s_crc_error_count = 0u;
    memset(&s_parser, 0, sizeof(s_parser));
    s_pan_deg = PAN_HOME_DEG;
    s_pan_moving = 0u;
    s_pan_direction = 0u;
    s_pan_steps_remaining = 0u;
    s_pan_current_steps = 0;
    s_pan_target_steps = 0;
    s_fault_flags = S31_FAULT_GIMBAL_UNHOMED;
    s_ultrasonic_cm = 0u;
    s_yaw_cdeg = 0;
    reset_pi(&s_pi_a);
    reset_pi(&s_pi_b);

    nvm_load();

    uart3_init();
    debug_uart1_init();
    ultrasonic_init();
    line_sensor_init();
    tsl1401_init();
    OLED_Init();
    debug_uart1_send(
        "Goumang C30D diag v3.10.0-pan540-audit, UART1=115200 output-only\r\n");

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    if (SysVal.HardWare_Ver == V1_1)
    {
        invMSInit();
        s_imu_ready = 1u;
    }
    else
    {
        (void)MPU6050_initialize();
        s_imu_ready = 1u;
    }
    s_last_yaw_tick = xTaskGetTickCount();

    s_ultrasonic_ready = 1u;
    stop_motors_now();
}

static bool bt_action_is_motion(uint8_t action)
{
    return action == TD_BT_ACTION_FORWARD ||
           action == TD_BT_ACTION_BACKWARD ||
           action == TD_BT_ACTION_TURN_LEFT ||
           action == TD_BT_ACTION_TURN_RIGHT ||
           action == TD_BT_ACTION_ARC_FORWARD_LEFT ||
           action == TD_BT_ACTION_ARC_FORWARD_RIGHT ||
           action == TD_BT_ACTION_ARC_BACKWARD_LEFT ||
           action == TD_BT_ACTION_ARC_BACKWARD_RIGHT;
}

static void bt_motion_for_speed(uint8_t action, uint8_t speed,
                                int8_t *left_percent,
                                int8_t *right_percent)
{
    uint8_t slow = (uint8_t)((speed * 3u + 2u) / 5u);

    switch (action)
    {
        case TD_BT_ACTION_FORWARD:
            *left_percent = (int8_t)speed;
            *right_percent = (int8_t)speed;
            break;
        case TD_BT_ACTION_BACKWARD:
            *left_percent = (int8_t)-speed;
            *right_percent = (int8_t)-speed;
            break;
        case TD_BT_ACTION_TURN_LEFT:
            *left_percent = (int8_t)-speed;
            *right_percent = (int8_t)speed;
            break;
        case TD_BT_ACTION_TURN_RIGHT:
            *left_percent = (int8_t)speed;
            *right_percent = (int8_t)-speed;
            break;
        case TD_BT_ACTION_ARC_FORWARD_LEFT:
            *left_percent = (int8_t)speed;
            *right_percent = (int8_t)slow;
            break;
        case TD_BT_ACTION_ARC_FORWARD_RIGHT:
            *left_percent = (int8_t)slow;
            *right_percent = (int8_t)speed;
            break;
        case TD_BT_ACTION_ARC_BACKWARD_LEFT:
            *left_percent = (int8_t)-speed;
            *right_percent = (int8_t)-slow;
            break;
        case TD_BT_ACTION_ARC_BACKWARD_RIGHT:
            *left_percent = (int8_t)-slow;
            *right_percent = (int8_t)-speed;
            break;
        default:
            *left_percent = 0;
            *right_percent = 0;
            break;
    }
}

static bool select_drive_command(uint32_t now,
                                 int8_t *left_percent,
                                 int8_t *right_percent,
                                 uint8_t *source)
{
    bool selected = false;

    taskENTER_CRITICAL();
    if (s_motion_stop_latched)
    {
        taskEXIT_CRITICAL();
        *left_percent = 0;
        *right_percent = 0;
        *source = 0u;
        return false;
    }
    bool s31_recent =
        s_armed &&
        (uint32_t)(now - s_last_drive_tick) <= S31_DRIVE_TIMEOUT_MS;
    bool line_recent =
        s_line_enabled &&
        (s_line_bt_mode ||
         (uint32_t)(now - s_line_last_cmd_tick) <= S31_LINE_TIMEOUT_MS);
    bool s31_nonzero_recent =
        s31_recent &&
        (s_left_percent != 0 || s_right_percent != 0) &&
        (uint32_t)(now - s_last_nonzero_drive_tick) <=
            S31_DRIVE_TIMEOUT_MS;
    bool bt_recent =
        s_bt_armed &&
        (uint32_t)(now - s_bt_last_command_tick) <=
            TD_BT_COMMAND_TIMEOUT_MS;

    if (s_bt_armed && !bt_recent)
    {
        s_bt_armed = 0u;
        s_bt_direction_action = TD_BT_ACTION_NONE;
    }

    if (line_recent)
    {
        *left_percent = s_line_left_percent;
        *right_percent = s_line_right_percent;
        *source = 3u;
        selected = true;
    }
    else if (s31_nonzero_recent)
    {
        *left_percent = s_left_percent;
        *right_percent = s_right_percent;
        *source = 1u;
        selected = true;
    }
    else if (bt_recent)
    {
        *left_percent = s_bt_left_percent;
        *right_percent = s_bt_right_percent;
        *source = 2u;
        selected = true;
    }
    taskEXIT_CRITICAL();

    if (!selected)
    {
        *left_percent = 0;
        *right_percent = 0;
        *source = 0u;
    }
    return selected;
}

bool s31_link_apply_bluetooth_command(const td_bt_command_t *command)
{
    TickType_t now = xTaskGetTickCount();
    int8_t left_percent;
    int8_t right_percent;
    uint8_t speed;

    if (command == NULL) {
        return false;
    }

    switch (command->action)
    {
        case TD_BT_ACTION_FORWARD:
        case TD_BT_ACTION_BACKWARD:
        case TD_BT_ACTION_TURN_LEFT:
        case TD_BT_ACTION_TURN_RIGHT:
        case TD_BT_ACTION_ARC_FORWARD_LEFT:
        case TD_BT_ACTION_ARC_FORWARD_RIGHT:
        case TD_BT_ACTION_ARC_BACKWARD_LEFT:
        case TD_BT_ACTION_ARC_BACKWARD_RIGHT:
            if (s_estop || EN == 0 || Voltage < 10.0f)
            {
                return false;
            }
            s_motion_stop_latched = 0u;
            s_bt_stop_holdoff_tick = 0u;
            if (command->explicit_speed)
            {
                int left_magnitude = command->left_percent < 0 ?
                    -command->left_percent : command->left_percent;
                int right_magnitude = command->right_percent < 0 ?
                    -command->right_percent : command->right_percent;
                speed = (uint8_t)(left_magnitude > right_magnitude ?
                                  left_magnitude : right_magnitude);
                left_percent = command->left_percent;
                right_percent = command->right_percent;
            }
            else
            {
                speed = s_bt_speed_percent;
                bt_motion_for_speed((uint8_t)command->action, speed,
                                    &left_percent, &right_percent);
            }
            s_line_enabled = 0u;
            s_line_bt_mode = 0u;
            s_line_left_percent = 0;
            s_line_right_percent = 0;
            taskENTER_CRITICAL();
            s_bt_direction_action = (uint8_t)command->action;
            s_bt_speed_percent = speed;
            s_bt_left_percent = left_percent;
            s_bt_right_percent = right_percent;
            s_bt_last_command_tick = (uint32_t)now;
            s_bt_armed = 1u;
            ++s_bt_command_count;
            taskEXIT_CRITICAL();
            return true;

        case TD_BT_ACTION_SPEED_UP:
        case TD_BT_ACTION_SPEED_DOWN:
            if (s_estop || EN == 0 || Voltage < 10.0f)
            {
                return false;
            }
            taskENTER_CRITICAL();
            if (command->action == TD_BT_ACTION_SPEED_UP)
            {
                s_bt_speed_percent =
                    s_bt_speed_percent <= 90u ?
                    (uint8_t)(s_bt_speed_percent + 10u) : 100u;
            }
            else
            {
                s_bt_speed_percent =
                    s_bt_speed_percent > 20u ?
                    (uint8_t)(s_bt_speed_percent - 10u) : 10u;
            }
            if (bt_action_is_motion(s_bt_direction_action))
            {
                bt_motion_for_speed(s_bt_direction_action,
                                    s_bt_speed_percent,
                                    &left_percent, &right_percent);
                s_bt_left_percent = left_percent;
                s_bt_right_percent = right_percent;
                s_bt_last_command_tick = (uint32_t)now;
                s_bt_armed = 1u;
            }
            ++s_bt_command_count;
            taskEXIT_CRITICAL();
            return true;

        case TD_BT_ACTION_STOP:
            s_motion_stop_latched = 1u;
            s_bt_stop_holdoff_tick = (uint32_t)now;
            s_line_enabled = 0u;
            s_line_bt_mode = 0u;
            s_line_left_percent = 0;
            s_line_right_percent = 0;
            taskENTER_CRITICAL();
            s_bt_armed = 0u;
            s_bt_left_percent = 0;
            s_bt_right_percent = 0;
            s_bt_direction_action = TD_BT_ACTION_NONE;
            s_left_percent = 0;
            s_right_percent = 0;
            s_armed = 0u;
            ++s_bt_command_count;
            taskEXIT_CRITICAL();
            stop_motors_now();
            return true;

        case TD_BT_ACTION_ESTOP:
            s_line_enabled = 0u;
            s_line_bt_mode = 0u;
            s_line_left_percent = 0;
            s_line_right_percent = 0;
            taskENTER_CRITICAL();
            s_estop = 1u;
            s_fault_flags |= S31_FAULT_ESTOP;
            s_bt_armed = 0u;
            s_bt_left_percent = 0;
            s_bt_right_percent = 0;
            s_bt_direction_action = TD_BT_ACTION_NONE;
            ++s_bt_command_count;
            taskEXIT_CRITICAL();
            chassis_set_pwm(0, 0);
            return true;

        case TD_BT_ACTION_CLEAR_ESTOP:
            s_line_enabled = 0u;
            s_line_bt_mode = 0u;
            s_line_left_percent = 0;
            s_line_right_percent = 0;
            taskENTER_CRITICAL();
            s_estop = 0u;
            s_fault_flags &= (uint8_t)~S31_FAULT_ESTOP;
            s_motion_stop_latched = 1u;
            s_left_percent = 0;
            s_right_percent = 0;
            s_last_drive_tick = (uint32_t)now;
            s_bt_armed = 0u;
            s_bt_left_percent = 0;
            s_bt_right_percent = 0;
            s_bt_direction_action = TD_BT_ACTION_NONE;
            ++s_bt_command_count;
            taskEXIT_CRITICAL();
            return true;

        case TD_BT_ACTION_LINE_FOLLOW:
            if (s_estop || EN == 0 || Voltage < 10.0f)
            {
                return false;
            }
            if (!s_line_enabled)
            {
                line_motion_init(&s_line_motion);
            }
            s_motion_stop_latched = 0u;
            s_bt_stop_holdoff_tick = 0u;
            s_line_enabled = 1u;
            s_line_bt_mode = 1u;
            s_line_speed_percent = (uint8_t)param_get(1u);
            s_line_last_cmd_tick = (uint32_t)now;
            s_bt_armed = 0u;
            s_bt_left_percent = 0;
            s_bt_right_percent = 0;
            s_bt_direction_action = TD_BT_ACTION_NONE;
            ++s_bt_command_count;
            return true;

        case TD_BT_ACTION_IGNORE:
            return true;

        default:
            return false;
    }
}

void s31_link_get_status(S31_Status_t *status)
{
    TickType_t now;
    int8_t left_percent;
    int8_t right_percent;
    uint8_t source;
    bool active;

    if (status == NULL) {
        return;
    }
    now = xTaskGetTickCount();
    active = select_drive_command(now, &left_percent, &right_percent,
                                  &source);

    taskENTER_CRITICAL();
    status->left_percent = left_percent;
    status->right_percent = right_percent;
    status->left_speed_mm_s = (int16_t)(MOTOR_A.Encoder * 1000.0f);
    status->right_speed_mm_s = (int16_t)(MOTOR_B.Encoder * 1000.0f);
    status->yaw_cdeg = s_yaw_cdeg;
    status->distance_cm = s_ultrasonic_cm == 0u ? 0xFFFFu : s_ultrasonic_cm;
    status->battery_mv = Voltage > 0.0f && Voltage < 30.0f ?
                         (uint16_t)(Voltage * 1000.0f) : 0u;
    status->fault_flags = s_fault_flags;
    status->s31_online =
        s_armed &&
        (uint32_t)(now - s_last_drive_tick) <= S31_DRIVE_TIMEOUT_MS;
    status->bluetooth_active =
        active && source == 2u && !s_estop;
    status->bluetooth_commands = s_bt_command_count;
    status->line_bits = s_line_bits;
    status->line_enabled = s_line_enabled;
    status->line_left_percent = s_line_left_percent;
    status->line_right_percent = s_line_right_percent;
    status->robot_state =
        s_estop ? S31_ROBOT_ESTOP :
        (active ? S31_ROBOT_ACTIVE : S31_ROBOT_IDLE);
    taskEXIT_CRITICAL();
}

void s31_control_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t now;
    uint32_t age;
    uint32_t telemetry_elapsed = S31_TELEMETRY_PERIOD_MS;
    uint32_t ultrasonic_elapsed = S31_ULTRASONIC_PERIOD_MS;
    uint32_t imu_elapsed = 200u;
    uint32_t oled_elapsed = S31_OLED_PERIOD_MS;
    float battery_sample;
    float left_target;
    float right_target;
    int8_t active_left;
    int8_t active_right;
    uint8_t active_source;
    bool drive_active;

    (void)argument;

    while (1)
    {
        vTaskDelayUntil(&last_wake, F2T(RATE_100_HZ));
        ++s_main_loop_count;
        if (SysVal.Time_count < 3000)
        {
            ++SysVal.Time_count;
        }
        now = xTaskGetTickCount();
        s_main_stage = 'R';

        poll_rx();
        nvm_poll();
        if (s_pan_boot_moving && s_pan_moving == 0u)
        {
            s_pan_boot_moving = 0u;
        }
        if (s_pan_boot_pending && !s_estop && Voltage >= 10.0f &&
            EN != 0 && Flag_Stop == 0u && !s_active_source &&
            !s_line_enabled && !s_left_percent && !s_right_percent &&
            !s_bt_left_percent && !s_bt_right_percent && !s_test_active &&
            pan_move_to_steps(0) == 0u)
        {
            s_pan_boot_moving = s_pan_moving;
            s_pan_boot_pending = 0u;
        }
        s_main_stage = 'U';
        chassis_get_velocity();
        s_main_stage = 'Y';
        update_yaw(now);
        line_sensor_update((uint32_t)now);

        if (s_armed)
        {
            age = (uint32_t)(now - s_last_drive_tick);
        }
        else
        {
            age = 0xFFFFFFFFu;
        }

        if (s_armed && age > S31_DRIVE_TIMEOUT_MS)
        {
            s_fault_flags |= S31_FAULT_CMD_TIMEOUT;
            s_armed = 0u;
            s_left_percent = 0;
            s_right_percent = 0;
        }
        if (s_line_enabled && !s_line_bt_mode &&
            (uint32_t)(now - s_line_last_cmd_tick) > S31_LINE_TIMEOUT_MS)
        {
            s_fault_flags |= S31_FAULT_CMD_TIMEOUT;
            s_line_enabled = 0u;
            s_line_bt_mode = 0u;
            s_line_left_percent = 0;
            s_line_right_percent = 0;
        }

        drive_active = select_drive_command(
            (uint32_t)now, &active_left, &active_right, &active_source);
        if (!s_line_enabled &&
            line_motion_ultrasonic_guard(
                s_ultrasonic_ready != 0u && s_ultrasonic_cm != 0u,
                s_ultrasonic_cm, active_left, active_right) ==
                LINE_GUARD_STOP)
        {
            drive_active = false;
            active_left = 0;
            active_right = 0;
            active_source = 0u;
        }
        s_active_left_percent = drive_active ? active_left : 0;
        s_active_right_percent = drive_active ? active_right : 0;
        s_active_source = drive_active ? active_source : 0u;

        if (s_estop)
        {
            stop_motors_now();
        }
        else if (chassis_turn_off(Voltage) != 0u)
        {
            stop_motors_now();
        }
        else if (!drive_active)
        {
            halt_motor_output();
        }
        else
        {
            left_target = (float)active_left *
                          (s_max_speed_mps / 100.0f);
            right_target = (float)active_right *
                           (s_max_speed_mps / 100.0f);

            MOTOR_A.Target = left_target;
            MOTOR_B.Target = right_target;
            MOTOR_C.Target = 0.0f;
            MOTOR_D.Target = 0.0f;

            /*
             * The official L150PRO V2 source feeds m/s directly into the
             * 700/700 incremental PI loop. Scaling both values by 1000 makes
             * the effective gains 1000x larger and saturates instantly.
             */
            MOTOR_A.Motor_Pwm =
                pi_step(&s_pi_a, MOTOR_A.Encoder, MOTOR_A.Target);
            MOTOR_B.Motor_Pwm =
                pi_step(&s_pi_b, MOTOR_B.Encoder, MOTOR_B.Target);
            MOTOR_C.Motor_Pwm = 0.0f;
            MOTOR_D.Motor_Pwm = 0.0f;

            /* Tank_Car polarity follows the official WHEELTEC source. */
            chassis_set_pwm((int)(-MOTOR_A.Motor_Pwm),
                            (int)MOTOR_B.Motor_Pwm);
            if (s_estop)
            {
                stop_motors_now();
            }
        }
        s_main_stage = 'M';

        telemetry_elapsed += 10u;
        if (telemetry_elapsed >= S31_TELEMETRY_PERIOD_MS)
        {
            s_main_stage = 'T';
            telemetry_elapsed = 0u;
            s_main_stage = 'B';
            battery_sample = sample_battery_voltage();
            if (Voltage <= 0.0f)
            {
                Voltage = battery_sample;
            }
            else
            {
                Voltage = Voltage * 0.80f + battery_sample * 0.20f;
            }
            if (s_rx_overflow_count != 0u)
            {
                s_fault_flags |= S31_FAULT_UART_OVERFLOW;
            }
            if (s_ultrasonic_ready && s_ultrasonic_cm == 0u)
            {
                s_fault_flags |= S31_FAULT_ULTRA_TIMEOUT;
            }
            else
            {
                s_fault_flags &= (uint8_t)~S31_FAULT_ULTRA_TIMEOUT;
            }
            s_main_stage = 'C';
            send_chassis_state();
            s_main_stage = 'N';
            send_sensor_state();
            s_main_stage = 'F';
            send_safety_state(now);
            s_main_stage = 'G';
            send_line_state();
            s_main_stage = 'M';
            send_motor_state();
            s_main_stage = 'S';
        }

        imu_elapsed += 10u;
        if (imu_elapsed >= 200u)
        {
            imu_elapsed = 0u;
            send_imu_state();
        }

        ultrasonic_elapsed += 10u;
        if (ultrasonic_elapsed >= S31_ULTRASONIC_PERIOD_MS)
        {
            s_main_stage = 'L';
            ultrasonic_elapsed = 0u;
            s_ultrasonic_cm = ultrasonic_read_filtered();
            s_main_stage = 'S';
        }

        oled_elapsed += 10u;
        if (oled_elapsed >= S31_OLED_PERIOD_MS)
        {
            s_main_stage = 'O';
            oled_elapsed = 0u;
            if (((uint32_t)now / pdMS_TO_TICKS(2000u)) & 1u)
            {
                oled_refresh_drive_diagnostic();
            }
            else
            {
                oled_refresh_status(now);
            }
            s_main_stage = 'H';
        }

    }
}

void s31_debug_monitor(void)
{
    static uint32_t last_loop_count;
    static TickType_t last_change_tick;
    static TickType_t last_report_tick;
    TickType_t now = xTaskGetTickCount();
    uint32_t loops = s_main_loop_count;

    if (loops != last_loop_count)
    {
        last_loop_count = loops;
        last_change_tick = now;
        return;
    }
    if ((uint32_t)(now - last_change_tick) < pdMS_TO_TICKS(500u) ||
        (uint32_t)(now - last_report_tick) < pdMS_TO_TICKS(500u))
    {
        return;
    }
    last_report_tick = now;
    if (s_uart3_txe_timeouts != 0u || s_uart3_tc_timeouts != 0u ||
        s_battery_adc_timeouts != 0u)
    {
        debug_uart1_send("Goumang STALL\r\n");
    }
    (void)s_main_stage;
    (void)USART3->SR;
    (void)GPIOD->MODER;
}

bool s31_link_estop_active(void)
{
    return s_estop != 0u;
}

bool s31_link_is_armed(void)
{
    return s_armed != 0u;
}

uint16_t s31_link_ultrasonic_cm(void)
{
    return s_ultrasonic_cm;
}

int16_t s31_link_yaw_cdeg(void)
{
    return s_yaw_cdeg;
}

uint16_t s31_link_pan_deg(void)
{
    return s_pan_deg;
}

