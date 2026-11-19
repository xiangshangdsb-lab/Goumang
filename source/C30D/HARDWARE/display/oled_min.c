#include "oled_min.h"

#include <string.h>

#include "delay.h"

#define OLED_CMD                 0u
#define OLED_DATA                1u

#define OLED_RST_LOW()           GPIO_ResetBits(GPIOD, GPIO_Pin_12)
#define OLED_RST_HIGH()          GPIO_SetBits(GPIOD, GPIO_Pin_12)
#define OLED_DC_LOW()            GPIO_ResetBits(GPIOD, GPIO_Pin_11)
#define OLED_DC_HIGH()           GPIO_SetBits(GPIOD, GPIO_Pin_11)
#define OLED_SCL_LOW()           GPIO_ResetBits(GPIOD, GPIO_Pin_14)
#define OLED_SCL_HIGH()          GPIO_SetBits(GPIOD, GPIO_Pin_14)
#define OLED_SDA_LOW()           GPIO_ResetBits(GPIOD, GPIO_Pin_13)
#define OLED_SDA_HIGH()          GPIO_SetBits(GPIOD, GPIO_Pin_13)

extern const unsigned char oled_asc2_1206[95][12];

u8 OLED_GRAM[128][8];

static void oled_write_byte(u8 data, u8 is_data)
{
    u8 bit;

    if (is_data)
    {
        OLED_DC_HIGH();
    }
    else
    {
        OLED_DC_LOW();
    }

    for (bit = 0u; bit < 8u; ++bit)
    {
        OLED_SCL_LOW();
        if (data & 0x80u)
        {
            OLED_SDA_HIGH();
        }
        else
        {
            OLED_SDA_LOW();
        }
        OLED_SCL_HIGH();
        data <<= 1;
    }
    OLED_DC_HIGH();
}

void OLED_Refresh_Gram(void)
{
    u8 page;
    u8 column;

    for (page = 0u; page < 8u; ++page)
    {
        oled_write_byte((u8)(0xB0u + page), OLED_CMD);
        oled_write_byte(0x00u, OLED_CMD);
        oled_write_byte(0x10u, OLED_CMD);
        for (column = 0u; column < 128u; ++column)
        {
            oled_write_byte(OLED_GRAM[column][page], OLED_DATA);
        }
    }
}

void OLED_Clear(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    OLED_Refresh_Gram();
}

static void oled_draw_point(u8 x, u8 y, u8 on)
{
    u8 page;
    u8 mask;

    if (x >= 128u || y >= 64u)
    {
        return;
    }

    page = (u8)(7u - y / 8u);
    mask = (u8)(1u << (7u - (y % 8u)));
    if (on)
    {
        OLED_GRAM[x][page] |= mask;
    }
    else
    {
        OLED_GRAM[x][page] &= (u8)~mask;
    }
}

void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode)
{
    u8 row;
    u8 bit;
    u8 y_start = y;
    u8 data;

    (void)size;
    if (chr < ' ' || chr > '~')
    {
        chr = '?';
    }
    chr = (u8)(chr - ' ');

    for (row = 0u; row < 12u; ++row)
    {
        data = oled_asc2_1206[chr][row];
        for (bit = 0u; bit < 8u; ++bit)
        {
            if (data & 0x80u)
            {
                oled_draw_point(x, y, mode);
            }
            else
            {
                oled_draw_point(x, y, (u8)!mode);
            }
            data <<= 1;
            ++y;
            if ((u8)(y - y_start) == 12u)
            {
                y = y_start;
                ++x;
                break;
            }
        }
    }
}

static u32 oled_power10(u8 exponent)
{
    u32 result = 1u;

    while (exponent-- > 0u)
    {
        result *= 10u;
    }
    return result;
}

void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size)
{
    u8 index;
    u8 digit;
    u8 visible = 0u;

    for (index = 0u; index < len; ++index)
    {
        digit = (u8)((num / oled_power10((u8)(len - index - 1u))) % 10u);
        if (!visible && index < (u8)(len - 1u))
        {
            if (digit == 0u)
            {
                OLED_ShowChar((u8)(x + 4u * index), y, ' ', size, 1u);
                continue;
            }
            visible = 1u;
        }
        OLED_ShowChar((u8)(x + 4u * index), y,
                      (u8)(digit + '0'), size, 1u);
    }
}

void OLED_ShowString(u8 x, u8 y, const u8 *text)
{
    while (text != NULL && *text != '\0')
    {
        if (x > 122u)
        {
            x = 0u;
            y = (u8)(y + 16u);
        }
        OLED_ShowChar(x, y, *text, 12u, 1u);
        x = (u8)(x + 8u);
        ++text;
    }
}

void OLED_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 |
                    GPIO_Pin_13 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &gpio);

    OLED_RST_LOW();
    delay_ms(100u);
    OLED_RST_HIGH();

    oled_write_byte(0xAEu, OLED_CMD);
    oled_write_byte(0xD5u, OLED_CMD);
    oled_write_byte(0x80u, OLED_CMD);
    oled_write_byte(0xA8u, OLED_CMD);
    oled_write_byte(0x3Fu, OLED_CMD);
    oled_write_byte(0xD3u, OLED_CMD);
    oled_write_byte(0x00u, OLED_CMD);
    oled_write_byte(0x40u, OLED_CMD);
    oled_write_byte(0x8Du, OLED_CMD);
    oled_write_byte(0x14u, OLED_CMD);
    oled_write_byte(0x20u, OLED_CMD);
    oled_write_byte(0x02u, OLED_CMD);
    oled_write_byte(0xA1u, OLED_CMD);
    oled_write_byte(0xC0u, OLED_CMD);
    oled_write_byte(0xDAu, OLED_CMD);
    oled_write_byte(0x12u, OLED_CMD);
    oled_write_byte(0x81u, OLED_CMD);
    oled_write_byte(0xEFu, OLED_CMD);
    oled_write_byte(0xD9u, OLED_CMD);
    oled_write_byte(0xF1u, OLED_CMD);
    oled_write_byte(0xDBu, OLED_CMD);
    oled_write_byte(0x30u, OLED_CMD);
    oled_write_byte(0xA4u, OLED_CMD);
    oled_write_byte(0xA6u, OLED_CMD);
    oled_write_byte(0xAFu, OLED_CMD);

    OLED_Clear();
}
