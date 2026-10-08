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
 * a 96 MHz system clock.
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
 * 16 MHz / 16 = 1 MHz
 * 1 MHz * 192 = 192 MHz
 * 200 MHZ / 2 = 96 MHz (system clock)
 * 192 MHZ / 4 = 48 MHZ (USB clock)
 *
 * APB1 is configured to 48 MHz and APB2 is configured
 * to 96 MHz.
 */
void clock_96MHz(void);

/** CPU clock frequency in Hz */
#define CPU_HZ  96000000.0f

/**
 * @brief Starts the DWT cycle counter
 *
 * This function enables the trace unit, resets the cycle counter
 * to zero and starts it. DWT->CYCCNT then increases by one on every
 * CPU clock cycle. It must be called once at startup, before
 * bmi_init(), because the BMI270 interrupt handler reads the counter.
 */
void dwt_init(void);

#endif /* CLK_SPEED_H */
