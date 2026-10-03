/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file clk_speed.h
 * @brief Clock configuration for the STM32F411
 *
 * This file contains the function declaration for configuring
 * the STM32F411 system clock. The external 16 MHz crystal is
 * used as the clock source and the PLL is configured to generate
 * a 100 MHz system clock.
 *
 * @author Mohammed Kassem
 * @date 03/10/2026
 *
 */

#ifndef CLK_SPEED_H
#define CLK_SPEED_H

/**
 * @brief Configures the STM32F411 system clock to 100 MHz
 *
 * This function enables the external 16 MHz crystal (HSE),
 * configures the PLL, and switches the system clock to the
 * PLL output.
 *
 * Clock calculation:
 * 16 MHz / 16 = 1 MHz
 * 1 MHz * 200 = 200 MHz
 * 200 MHz / 2 = 100 MHz
 *
 * APB1 is configured to 50 MHz and APB2 is configured
 * to 100 MHz.
 */
void clock_100MHz(void);

#endif /* CLK_SPEED_H */
