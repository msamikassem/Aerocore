/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file uart.c
 * @brief USART2 driver for the radio receiver on the STM32F411
 *
 * This file implements a simple polling driver for USART2. The radio
 * receiver is connected to PA2 (TX) and PA3 (RX), both using alternate
 * function AF7. The USART runs at 420000 baud with 8 data bits, no parity
 * and 1 stop bit. The baud rate register is calculated from the 16 MHz
 * clock the USART runs from at startup (HSI).
 *
 * @author Mohammed Kassem
 * @date 10/09/2026
 *
 */

#include "stm32f4xx.h"
#include "uart.h"
#include <stdint.h>


// Clock and baud rate
#define SYS_CLK     (48000000U)    // APB1 clock after clock_96MHz()
#define USART_CLK   (SYS_CLK)
#define BAUDRATE    (420000U)

// Pins (GPIOA)
#define PIN_TX      2U      // PA2 = USART2_TX
#define PIN_RX      3U      // PA3 = USART2_RX
#define AF_USART2   7U      // alternate function 7 (AF07)


void UART2_init(void)
{
    // Provide CLK to GPIOA (AHB1) and USART2 (APB1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Set PA2 (TX) and PA3 (RX) to alternate function mode (10)
    GPIOA->MODER &= ~(3U << (PIN_TX * 2U));
    GPIOA->MODER |=  (2U << (PIN_TX * 2U));
    GPIOA->MODER &= ~(3U << (PIN_RX * 2U));
    GPIOA->MODER |=  (2U << (PIN_RX * 2U));

    // Select AF7 (USART2) on AFRL, 4 bits per pin
    GPIOA->AFR[0] &= ~(0xFU << (PIN_TX * 4U));
    GPIOA->AFR[0] |=  (AF_USART2 << (PIN_TX * 4U));
    GPIOA->AFR[0] &= ~(0xFU << (PIN_RX * 4U));
    GPIOA->AFR[0] |=  (AF_USART2 << (PIN_RX * 4U));

    // Baud rate = USART_CLK / BRR (rounded to the nearest integer)
    USART2->BRR = (uint16_t)((USART_CLK + BAUDRATE / 2U) / BAUDRATE);

    // Enable transmitter, receiver and the USART
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}


uint8_t UART2_read(void)
{
    // Wait until a byte has been received (RXNE flag)
    while (!(USART2->SR & USART_SR_RXNE))
    {
    }

    // Reading DR also clears the RXNE flag
    return (uint8_t)USART2->DR;
}
