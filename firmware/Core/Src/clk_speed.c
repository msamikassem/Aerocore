/**
 * @file clk_speed.c
 * @brief STM32F411 clock configuration
 *
 * This file implements the clock configuration for the STM32F411.
 * The external 16 MHz crystal is used as the HSE source for the PLL.
 *
 * PLL configuration:
 * 16 MHz / 16 = 1 MHz
 * 1 MHz * 192 = 192 MHz
 * 200 MHZ / 2 = 96 MHz (system clock)
 * 192 MHZ / 4 = 48 MHZ (USB clock)
 * APB1 is configured to 48 MHz and APB2 is configured to 96 MHz.
 *
 * @author Mohammed Kassem
 * @date 10/03/2026
 *
 */


#include "stm32f411xe.h"
#include "clk_speed.h"

void clock_96MHz(void)
{
    //Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    //Flash configuration for 96 MHz
    FLASH->ACR = FLASH_ACR_ICEN|FLASH_ACR_DCEN|FLASH_ACR_LATENCY_3WS;

    // APB1 = 48 MHz, APB2 = 96 MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    // Configure PLL
    // Use 16 MHz external crystal and multiply it to 96 MHz:
    // 16 MHz / 16 = 1 MHz
    // 1 MHz * 192 = 192 MHz
    // 192 MHz / 2 = 96 MHz  (system clock)
    // 192 MHz / 4 = 48 MHz  (USB clock)
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) | (192 << RCC_PLLCFGR_PLLN_Pos) |(0 << RCC_PLLCFGR_PLLP_Pos) |(4 << RCC_PLLCFGR_PLLQ_Pos) |RCC_PLLCFGR_PLLSRC_HSE;

    //Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    //Switch system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}


void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
