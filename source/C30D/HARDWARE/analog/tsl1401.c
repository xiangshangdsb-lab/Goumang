#include "tsl1401.h"

#include <stddef.h>

#include "delay.h"

static uint32_t s_cycles_per_us;

static bool adc_read(uint16_t *value)
{
    uint32_t start;
    uint32_t timeout_cycles;

    if (value == NULL || s_cycles_per_us == 0u)
    {
        return false;
    }
    timeout_cycles = s_cycles_per_us * 1000u;
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
    ADC_SoftwareStartConv(ADC1);
    start = DWT->CYCCNT;
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_cycles)
        {
            return false;
        }
    }
    *value = ADC_GetConversionValue(ADC1);
    return true;
}

void tsl1401_init(void)
{
    GPIO_InitTypeDef gpio;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cycles_per_us = SystemCoreClock / 1000000u;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                           RCC_AHB1Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = TSL1401_AO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(TSL1401_AO_PORT, &gpio);

    gpio.GPIO_Pin = TSL1401_CLK_PIN | TSL1401_SI_PIN;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(TSL1401_CLK_PORT, &gpio);
    GPIO_Init(TSL1401_SI_PORT, &gpio);
    GPIO_ResetBits(TSL1401_CLK_PORT, TSL1401_CLK_PIN);
    GPIO_ResetBits(TSL1401_SI_PORT, TSL1401_SI_PIN);

    ADC_RegularChannelConfig(ADC1, TSL1401_AO_CHANNEL, 1u,
                             ADC_SampleTime_15Cycles);
}

bool tsl1401_read(uint16_t pixels[TSL1401_PIXELS],
                  uint32_t *elapsed_us)
{
    uint32_t start;
    uint16_t i;

    if (pixels == NULL || s_cycles_per_us == 0u)
    {
        return false;
    }

    start = DWT->CYCCNT;
    GPIO_ResetBits(TSL1401_CLK_PORT, TSL1401_CLK_PIN);
    GPIO_ResetBits(TSL1401_SI_PORT, TSL1401_SI_PIN);
    delay_us(2u);
    GPIO_SetBits(TSL1401_SI_PORT, TSL1401_SI_PIN);
    delay_us(2u);
    GPIO_ResetBits(TSL1401_CLK_PORT, TSL1401_CLK_PIN);
    delay_us(2u);
    GPIO_SetBits(TSL1401_CLK_PORT, TSL1401_CLK_PIN);
    delay_us(2u);
    GPIO_ResetBits(TSL1401_SI_PORT, TSL1401_SI_PIN);
    delay_us(2u);

    ADC_RegularChannelConfig(ADC1, TSL1401_AO_CHANNEL, 1u,
                             ADC_SampleTime_15Cycles);
    for (i = 0u; i < TSL1401_PIXELS; ++i)
    {
        GPIO_ResetBits(TSL1401_CLK_PORT, TSL1401_CLK_PIN);
        delay_us(2u);
        if (!adc_read(&pixels[i]))
        {
            return false;
        }
        GPIO_SetBits(TSL1401_CLK_PORT, TSL1401_CLK_PIN);
        delay_us(1u);
    }

    if (elapsed_us != NULL)
    {
        *elapsed_us =
            (uint32_t)(DWT->CYCCNT - start) / s_cycles_per_us;
    }
    return true;
}

