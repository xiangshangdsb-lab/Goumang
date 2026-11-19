#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TSL1401_PIXELS 128u

#define TSL1401_AO_PORT             GPIOA
#define TSL1401_AO_PIN              GPIO_Pin_4
#define TSL1401_AO_CHANNEL          ADC_Channel_4
#define TSL1401_CLK_PORT            GPIOC
#define TSL1401_CLK_PIN             GPIO_Pin_4
#define TSL1401_SI_PORT             GPIOA
#define TSL1401_SI_PIN              GPIO_Pin_5

void tsl1401_init(void);
bool tsl1401_read(uint16_t pixels[TSL1401_PIXELS],
                  uint32_t *elapsed_us);

