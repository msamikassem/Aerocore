/****************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 ****************************************************************************/
/**
 * @file tim4_pwm.h
 * @brief TIM4 PWM configuration for the STM32F411
 *
 * This file contains the function declarations for configuring
 * TIM4 PWM outputs on PB6, PB7, PB8 and PB9 for controlling
 * four ESCs.
 *
 * @author Mohammed Kassem
 * @date 10/10/2026
 *
 */

#ifndef TIM4_PWM_H
#define TIM4_PWM_H

#include <stdint.h>

/**
 * @brief Configures all four TIM4 PWM channels
 *
 * Configures PB6, PB7, PB8 and PB9 as alternate function outputs.
 * Sets the PWM frequency to 50 Hz and initializes all channels
 * to a 1000 us pulse width.
 */
void tim4_pwm(void);

/**
 * @brief Converts an input value to a PWM value
 *
 * @param input Value between 0.0 and 1.0
 * @return PWM value between 1000 and 2000
 */
uint32_t input_to_pwm(float input);

/**
 * @brief Changes the PWM value of a selected channel
 *
 * @param channel TIM4 channel (1-4)
 * @param value Input between 0.0 and 1.0
 */
void change_pwm(uint8_t channel, float value);

#endif /* TIM4_PWM_H */
