#include "bluetooth_link.h"

#include <stdio.h>
#include <string.h>

#include "bluetooth_control.h"
#include "../control/s31_link.h"
#include "system.h"

#define TD_BT_BAUDRATE          9600u
#define TD_BT_LINE_SIZE         40u
#define TD_BT_TX_RING_SIZE      256u
#define TD_BT_IDLE_DISPATCH_MS  10u

static uint8_t s_tx_ring[TD_BT_TX_RING_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static char s_line[TD_BT_LINE_SIZE];
static uint8_t s_line_length;
static TickType_t s_line_last_rx_tick;
static uint8_t s_in_app_frame;

static void bt_string(const char *text)
{
    uint16_t next;

    while (text != NULL && *text != '\0')
    {
        next = (uint16_t)((s_tx_head + 1u) & (TD_BT_TX_RING_SIZE - 1u));
        if (next == s_tx_tail)
        {
            return;
        }
        s_tx_ring[s_tx_head] = (uint8_t)*text++;
        s_tx_head = next;
    }
}

static void bt_send_telemetry(void)
{
    S31_Status_t status;
    char line[144];
    int yaw_abs;

    s31_link_get_status(&status);
    yaw_abs = status.yaw_cdeg < 0 ? -status.yaw_cdeg : status.yaw_cdeg;
    (void)snprintf(
        line, sizeof(line),
        "TD,ST=%u,S31=%u,BT=%u,L=%d,R=%d,LS=%d,RS=%d,"
        "Y=%c%d.%02d,V=%u,D=%u,F=%u,N=%lu\r\n",
        status.robot_state, status.s31_online, status.bluetooth_active,
        status.left_percent, status.right_percent,
        status.left_speed_mm_s, status.right_speed_mm_s,
        status.yaw_cdeg < 0 ? '-' : '+',
        yaw_abs / 100, yaw_abs % 100,
        status.battery_mv, status.distance_cm, status.fault_flags,
        (unsigned long)status.bluetooth_commands);
    bt_string(line);
}

static void bt_send_wheeltec_status(void)
{
    S31_Status_t status;
    char line[40];
    int battery_percent;
    int angle_deg;

    s31_link_get_status(&status);
    battery_percent = 0;
    if (status.battery_mv >= 10000u)
    {
        battery_percent = (status.battery_mv - 10000u) / 27u;
        if (battery_percent > 100)
        {
            battery_percent = 100;
        }
    }
    angle_deg = status.yaw_cdeg / 100;
    if (td_bt_format_wheeltec_status(
            status.left_speed_mm_s, status.right_speed_mm_s,
            battery_percent, angle_deg, line, sizeof(line)) > 0u)
    {
        bt_string(line);
    }
}

static void bt_send_help(void)
{
    bt_string(
        "Goumang APP: A/E/C/G/Z, B/D/F/H arc, X/Y speed\r\n"
        "Goumang F42 B42 L30 R30 STOP ESTOP CLEAR ?\r\n");
}

static void bt_handle_command(const char *line)
{
    td_bt_command_t command;

    if (line[0] == '+' || strcmp(line, "CONNECTED") == 0)
    {
        return;
    }
    if (!td_bt_parse_command(line, &command))
    {
        bt_string("Goumang ERR CMD\r\n");
        return;
    }
    if (command.action == TD_BT_ACTION_HELP)
    {
        bt_send_help();
        return;
    }

    if (!s31_link_apply_bluetooth_command(&command))
    {
        bt_string("Goumang BLOCKED\r\n");
        return;
    }

    if (command.action == TD_BT_ACTION_ESTOP)
    {
        bt_string("Goumang ESTOP\r\n");
    }
    else if (command.action == TD_BT_ACTION_CLEAR_ESTOP)
    {
        bt_string("Goumang CLEAR\r\n");
    }
    else if (command.action == TD_BT_ACTION_STOP)
    {
        bt_string("Goumang STOP\r\n");
    }
    else if (command.action == TD_BT_ACTION_SPEED_UP)
    {
        bt_string("Goumang OK SPEED+\r\n");
    }
    else if (command.action == TD_BT_ACTION_SPEED_DOWN)
    {
        bt_string("Goumang OK SPEED-\r\n");
    }
    else if (command.action == TD_BT_ACTION_LINE_FOLLOW)
    {
        bt_string("Goumang LINE\r\n");
    }
    else if (command.action == TD_BT_ACTION_IGNORE)
    {
        return;
    }
    else
    {
        char reply[48];
        (void)snprintf(reply, sizeof(reply),
                       "Goumang OK L=%d R=%d\r\n",
                       command.left_percent, command.right_percent);
        bt_string(reply);
    }
}

static bool bt_is_single_command_char(char value)
{
    return (value >= 'A' && value <= 'K') ||
           (value >= 'X' && value <= 'Z') ||
           (value >= 'a' && value <= 'k') ||
           (value >= 'x' && value <= 'z');
}

static void bt_dispatch_line(void)
{
    uint8_t i;
    bool single_commands = true;

    if (s_line_length == 0u) {
        return;
    }
    s_line[s_line_length] = '\0';
    for (i = 0u; i < s_line_length; ++i) {
        if (!bt_is_single_command_char(s_line[i])) {
            single_commands = false;
            break;
        }
    }
    if (s_line_length > 1u && single_commands) {
        char one[2] = {0, 0};
        for (i = 0u; i < s_line_length; ++i) {
            one[0] = s_line[i];
            bt_handle_command(one);
        }
    }
    else {
        bt_handle_command(s_line);
    }
    s_line_length = 0u;
}

static void bt_poll_rx(TickType_t now)
{
    char value;

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        value = (char)(uint8_t)USART_ReceiveData(USART2);
        s_line_last_rx_tick = now;
        if (s_in_app_frame)
        {
            if (value == '}')
            {
                s_in_app_frame = 0u;
            }
            continue;
        }
        if (value == '{')
        {
            s_in_app_frame = 1u;
            s_line_length = 0u;
            continue;
        }
        if (value == '\r' || value == '\n')
        {
            if (s_line_length > 0u)
            {
                bt_dispatch_line();
            }
        }
        else if (value == '\b' || value == 0x7F)
        {
            if (s_line_length > 0u)
            {
                --s_line_length;
            }
        }
        else if (value >= ' ' && value <= '~')
        {
            if (s_line_length < (TD_BT_LINE_SIZE - 1u))
            {
                s_line[s_line_length++] = value;
            }
            else
            {
                s_line_length = 0u;
                bt_string("Goumang ERR LONG\r\n");
            }
        }
    }
}

static void bt_dispatch_if_idle(TickType_t now)
{
    if (!s_in_app_frame && s_line_length > 0u &&
        (uint32_t)(now - s_line_last_rx_tick) >= TD_BT_IDLE_DISPATCH_MS)
    {
        bt_dispatch_line();
    }
}

static void bt_poll_tx(void)
{
    if (s_tx_tail != s_tx_head &&
        USART_GetFlagStatus(USART2, USART_FLAG_TXE) != RESET)
    {
        USART_SendData(USART2, s_tx_ring[s_tx_tail]);
        s_tx_tail = (uint16_t)((s_tx_tail + 1u) &
                               (TD_BT_TX_RING_SIZE - 1u));
    }
}

void bluetooth_link_init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);

    uart.USART_BaudRate = TD_BT_BAUDRATE;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    USART_Cmd(USART2, ENABLE);

    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_line_length = 0u;
    s_line_last_rx_tick = xTaskGetTickCount();
    s_in_app_frame = 0u;
    bt_string("Goumang BT ready. ?=help\r\n");
}

void bluetooth_link_task(void *argument)
{
    TickType_t last_telemetry = xTaskGetTickCount();
    TickType_t last_wheeltec_status = xTaskGetTickCount();
    TickType_t now;

    (void)argument;
    while (1)
    {
        now = xTaskGetTickCount();
        bt_poll_rx(now);
        bt_dispatch_if_idle(now);
        bt_poll_tx();
        s31_debug_monitor();
        if ((uint32_t)(now - last_wheeltec_status) >= 200u)
        {
            last_wheeltec_status = now;
            bt_send_wheeltec_status();
        }
        if ((uint32_t)(now - last_telemetry) >= 1000u)
        {
            last_telemetry = now;
            bt_send_telemetry();
        }
        vTaskDelay(1);
    }
}
