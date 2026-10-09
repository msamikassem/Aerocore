/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file uart.h
 * @brief USART2 driver for the radio receiver on the STM32F411
 *
 * This file contains the function declarations for USART2, which is
 * used to read the radio receiver. The receiver is connected to
 * PA2 (TX) and PA3 (RX). The USART runs at 420000 baud with 8 data
 * bits, no parity and 1 stop bit.
 *
 * @author Mohammed Kassem
 * @date 10/09/2026
 *
 */

#ifndef UART_H
#define UART_H

/** Set to 1 by the receive interrupt when a new RC packet has been decoded. Clear it after use */
extern volatile uint8_t rc_ready;

#include <stdint.h>

/**
 * @brief Initializes USART2
 *
 * This function enables the GPIOA and USART2 clocks, sets PA2 and PA3
 * to alternate function 7, sets the baud rate to 420000 and turns on
 * the transmitter, the receiver and the USART. It must be called once
 * at startup before reading any data.
 */
void UART2_init(void);

/**
 * @brief Reads one byte from USART2
 *
 * This function waits (blocks) until a byte has been received and
 * then returns it.
 *
 * @return The received byte
 */
uint8_t UART2_read(void);

#endif /* UART_H */
