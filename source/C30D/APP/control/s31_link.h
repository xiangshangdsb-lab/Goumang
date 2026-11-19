#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../bluetooth/bluetooth_control.h"

/*
 * Goumang C30D <-> ESP32-S31 transport.
 *
 * C30D side:
 *   USART3_TX = PD8 -> CH9102F RXD -> UART3 USB-C
 *   USART3_RX = PD9 <- CH9102F TXD <- UART3 USB-C
 *
 * S31 side must use its USB 2.0 Type-A host connector and a CDC-ACM host
 * driver.  The S31 USB Full-speed Type-C connector is a USB device port.
 *
 * Bluetooth serial module:
 *   USART2_TX = PD5
 *   USART2_RX = PD6
 *   9600 8N1, ASCII commands documented in bluetooth_control.h
 */
#define S31_TASK_STACK              768u
#define S31_BAUDRATE                115200u

/* UART1 TYPE-C is a 115200 8N1 diagnostic output. Open-loop motor
 * commands were removed in v3.9.1 to stay within the MDK-Lite limit;
 * use the authenticated S31 controls instead. */

/* A valid 0x10 frame must refresh at least this often. */
#define S31_DRIVE_TIMEOUT_MS        500u
#define S31_TELEMETRY_PERIOD_MS     100u
#define S31_ULTRASONIC_PERIOD_MS    50u
#define S31_OLED_PERIOD_MS          500u

typedef struct
{
    int8_t left_percent;
    int8_t right_percent;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    int16_t yaw_cdeg;
    uint16_t distance_cm;
    uint16_t battery_mv;
    uint8_t robot_state;
    uint8_t fault_flags;
    uint8_t s31_online;
    uint8_t bluetooth_active;
    uint32_t bluetooth_commands;
    uint8_t line_bits;
    uint8_t line_enabled;
    int8_t line_left_percent;
    int8_t line_right_percent;
} S31_Status_t;

/*
 * WHEELTEC R3 high-spec tracked chassis.
 *
 * These are the official L150PRO Tank_Car baseline values, not generic
 * differential-car defaults:
 *   wheel spacing       0.235 m
 *   drive wheel diameter 0.043 m
 *   Hall encoder         13 PPR, 30:1 gearbox, x4 decode
 *   speed loop           KP = 700, KI = 700
 */
#define S31_R3_HIGH_SPEC_TRACK          1
#define S31_TRACK_WHEEL_SPACING_M       0.235f
#define S31_TRACK_WHEEL_DIAMETER_M      0.043f
#define S31_TRACK_ENCODER_PPR           13
#define S31_TRACK_GEAR_RATIO            30
#define S31_TRACK_ENCODER_MULTIPLIER    4
#define S31_TRACK_SPEED_KP              700.0f
#define S31_TRACK_SPEED_KI              700.0f

/*
 * Test-speed limit.  0.35 m/s equals the original RC_Velocity default
 * (350 mm/s).  Raise only after the tracked chassis has been tested safely.
 */
#define S31_MAX_WHEEL_SPEED_MPS     0.35f

/*
 * HC-SR04 default wiring.  Change both pins here if the physical harness
 * uses another free header.
 *   PC0 = TRIG output
 *   PC1 = ECHO input (use a divider/level shifter for the usual 5 V echo)
 */
#define S31_ULTRA_TRIG_PORT         GPIOC
#define S31_ULTRA_TRIG_PIN          GPIO_Pin_0
#define S31_ULTRA_ECHO_PORT         GPIOC
#define S31_ULTRA_ECHO_PIN          GPIO_Pin_1

/*
 * WHEELTEC LF04 four-channel infrared line sensor on H2.
 * Sensor order is DH1 (left) .. DH4 (right); black line reads low.
 *   DH1/O1 -> PA2 (H2 pin 12)
 *   DH2/O2 -> PA3 (H2 pin 10)
 *   DH3/O3 -> PC2 (H2 pin 11)
 *   DH4/O4 -> PC3 (H2 pin 9)
 *   VCC    -> 5V  (H2 pin 4)
 *   GND    -> GND (H2 pin 3)
 */
#define S31_LINE_DH1_PORT           GPIOA
#define S31_LINE_DH1_PIN            GPIO_Pin_2
#define S31_LINE_DH2_PORT           GPIOA
#define S31_LINE_DH2_PIN            GPIO_Pin_3
#define S31_LINE_DH3_PORT           GPIOC
#define S31_LINE_DH3_PIN            GPIO_Pin_2
#define S31_LINE_DH4_PORT           GPIOC
#define S31_LINE_DH4_PIN            GPIO_Pin_3
#define S31_LINE_TIMEOUT_MS         500u
#define S31_LINE_LOST_STOP_MS       600u
#define S31_LINE_BASE_PERCENT       30
#define S31_LINE_TURN_GAIN          12

/*
 * D36A gimbal stepper on C30D H2 reserved-I/O header.
 *   STEP -> PA6   (H2 pin 8, TIM13_CH1)
 *   DIR  -> PC5   (H2 pin 7)
 *   EN   -> PC10  (H2 pin 2)
 *   GND  -> GND   (H2 pin 18 or 3)
 *
 * D36A is configured for 16 microstep. A 1.8-degree 42 stepper therefore
 * needs 200 * 16 = 3200 STEP pulses per revolution. The UI exposes
 * 0..180 degrees, mapped to -90..+90 degrees around the 90-degree centre.
 */
#define S31_PAN_STEP_PORT           GPIOA
#define S31_PAN_STEP_PIN            GPIO_Pin_6
#define S31_PAN_STEP_PIN_SOURCE     GPIO_PinSource6
#define S31_PAN_STEP_AF             GPIO_AF_TIM13
#define S31_PAN_DIR_PORT            GPIOC
#define S31_PAN_DIR_PIN             GPIO_Pin_5
#define S31_PAN_EN_PORT             GPIOC
#define S31_PAN_EN_PIN              GPIO_Pin_10
#define S31_PAN_STEPS_PER_REV       3200u
#define S31_PAN_STEPS_PER_180_DEG   1600
#define S31_PAN_MOVE_RPM            30u
#define S31_PAN_EN_ACTIVE_LOW       0

#if !S31_R3_HIGH_SPEC_TRACK
#error "This firmware build is intentionally configured for the R3 high-spec tracked chassis."
#endif

void s31_link_init(void);
void s31_control_task(void *argument);
void s31_debug_monitor(void);
bool s31_link_apply_bluetooth_command(const td_bt_command_t *command);
void s31_link_get_status(S31_Status_t *status);

bool s31_link_estop_active(void);
bool s31_link_is_armed(void);
uint16_t s31_link_ultrasonic_cm(void);
int16_t s31_link_yaw_cdeg(void);
uint16_t s31_link_pan_deg(void);

