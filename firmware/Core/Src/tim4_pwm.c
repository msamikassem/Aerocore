/**
 * @file tim4_pwm.c
 * @brief TIM4 PWM configuration for STM32F411
 *
 * This file configures TIM4 to generate PWM signals on PB6, PB7,
 * PB8 and PB9 for controlling four ESCs.
 *
 * Timer configuration:
 * TIM4 clock = 96 MHz
 * 96 MHz / 96 = 1 MHz
 * 1 timer count = 1 us
 * 20 ms period = 50 Hz
 *
 * PWM range:
 * 1000 us = minimum command
 * 2000 us = maximum command
 *
 * @author Mohammed Kassem
 * @date 10/10/2026
 *
 */

#include "stm32f411xe.h"
#include "tim4_pwm.h"

#define SYS_CLK (96000000U)


void tim4_pwm(void)
{
    // Turn on GPIOB clock
    RCC->AHB1ENR |= (1U << 1);

    // Turn on TIM4 clock
    RCC->APB1ENR |= (1U << 2);

    // PB6, PB7, PB8 and PB9 -> Alternate Function mode
    GPIOB->MODER &= ~((3U << 12) | (3U << 14) |
                      (3U << 16) | (3U << 18));

    GPIOB->MODER |=  ((2U << 12) | (2U << 14) |
                      (2U << 16) | (2U << 18));

    // PB6, PB7, PB8 and PB9 -> AF2 (TIM4)
    GPIOB->AFR[0] &= ~(0xFU << 24 | 0xFU << 28);
    GPIOB->AFR[0] |=  ((2U << 24) | (2U << 28));

    GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB->AFR[1] |=  ((2U << 0) | (2U << 4));

    // Timer configuration

    // 96 MHz / 96 = 1 MHz
    // 1 timer count = 1 us
    TIM4->PSC = 95;

    // 20 ms period = 50 Hz
    TIM4->ARR = 19999;

    // Start all channels at 1 ms
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
    TIM4->CCR3 = 1000;
    TIM4->CCR4 = 1000;

    // Configure channels 1 and 2
    // CC1S / CC2S = 00
    TIM4->CCMR1 &= ~((3U << 0) | (3U << 8));

    // PWM mode 1
    TIM4->CCMR1 &= ~((7U << 4) | (7U << 12));
    TIM4->CCMR1 |=  ((6U << 4) | (6U << 12));

    // Enable preload for channels 1 and 2
    TIM4->CCMR1 |= (1U << 3) | (1U << 11);

    // Configure channels 3 and 4
    // CC3S / CC4S = 00
    TIM4->CCMR2 &= ~((3U << 0) | (3U << 8));

    // PWM mode 1
    TIM4->CCMR2 &= ~((7U << 4) | (7U << 12));
    TIM4->CCMR2 |=  ((6U << 4) | (6U << 12));

    // Enable preload for channels 3 and 4
    TIM4->CCMR2 |= (1U << 3) | (1U << 11);

    // Enable all four channel outputs
    TIM4->CCER |= (1U << 0)  | (1U << 4) |
                  (1U << 8)  | (1U << 12);

    // Enable ARR preload
    TIM4->CR1 |= (1U << 7);

    // Load timer settings
    TIM4->EGR |= (1U << 0);

    // Turn on TIM4
    TIM4->CR1 |= (1U << 0);
}

uint32_t input_to_pwm(float input)
{
    // Convert input from 0-1 to 1000-2000 us
    return 1000U + (uint32_t)(input * 1000.0f);
}

void change_pwm(uint8_t channel, float value)
{
    uint32_t pwm_value = input_to_pwm(value);

    if (channel == 1)
        TIM4->CCR1 = pwm_value;

    else if (channel == 2)
        TIM4->CCR2 = pwm_value;

    else if (channel == 3)
        TIM4->CCR3 = pwm_value;

    else if (channel == 4)
        TIM4->CCR4 = pwm_value;
}



