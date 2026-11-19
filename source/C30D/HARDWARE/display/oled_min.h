#pragma once

#include "sys.h"

extern u8 OLED_GRAM[128][8];

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh_Gram(void);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size, u8 mode);
void OLED_ShowNumber(u8 x, u8 y, u32 num, u8 len, u8 size);
void OLED_ShowString(u8 x, u8 y, const u8 *text);

