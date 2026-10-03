/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file usb_comm.h
 * @brief Simple send/receive functions for USB CDC (virtual COM port)
 *
 * Wraps the CubeMX generated USB CDC code so the rest of the project
 * only needs these functions. On the PC the board shows up as a serial
 * port (/dev/ttyACM0), so any serial monitor can talk to it.
 *
 * Setup (one time):
 * 1. Add usb_comm.c to Core/Src and usb_comm.h to Core/Inc
 * 2. In USB_DEVICE/App/usbd_cdc_if.c, inside CDC_Receive_FS (between
 *    USER CODE BEGIN 6 and END 6) add this line first:
 *        usb_rx_callback(Buf, *Len);
 *    and add #include "usb_comm.h" under USER CODE BEGIN INCLUDE
 * 3. Call usb_init() once at the start of main. It sets up HAL, the
 *    96 MHz clock and USB, so nothing else is needed before it
 *
 * @author Mohammed Kassem
 * @date 03/10/2026
 *
 */

#ifndef USB_COMM_H
#define USB_COMM_H

#include <stdint.h>

/**
 * @brief Sets up everything needed for USB in one call
 *
 * Call this once at the start of main. It does the following in order:
 * 1. HAL_Init()                 (HAL and SysTick)
 * 2. clock_96MHz()              (96 MHz core, exact 48 MHz for USB)
 * 3. SystemCoreClockUpdate()    (refresh the clock variable)
 * 4. HAL_InitTick()             (re-time SysTick for 96 MHz)
 * 5. MX_USB_DEVICE_Init()       (USB pins, clock, interrupt, CDC class)
 *
 * Do not call HAL_Init() or clock_96MHz() again in main.
 */
void usb_init(void);

/**
 * @brief Checks if a PC is connected and has opened the USB port
 *
 * @return 1 if the USB link is up, 0 otherwise
 */
uint8_t usb_connected(void);

/**
 * @brief Sends raw bytes to the PC
 *
 * The data is copied into an internal buffer, so the buffer you pass
 * can be a local variable. Large data is sent in chunks. If the PC is
 * not connected or stops reading, the function gives up after a short
 * timeout instead of blocking forever.
 *
 * @param data Pointer to the bytes to send
 * @param len  Number of bytes to send
 * @return Number of bytes actually sent
 */
uint16_t usb_send(const uint8_t *data, uint16_t len);

/**
 * @brief Sends a null terminated string to the PC
 *
 * @param str String to send, for example "hello\r\n"
 */
void usb_print(const char *str);

/**
 * @brief Sends formatted text to the PC, like printf
 *
 * Example: usb_printf("gx=%d gy=%d\r\n", gx, gy);
 *
 * Integers and strings work. Floats print nothing unless float printf
 * is enabled in the linker settings (nano.specs does not support it).
 * Output is limited to 128 characters per call.
 *
 * @param fmt printf style format string
 */
void usb_printf(const char *fmt, ...);

/**
 * @brief Returns how many received bytes are waiting to be read
 *
 * @return Number of bytes in the receive buffer
 */
uint16_t usb_available(void);

/**
 * @brief Reads one received byte
 *
 * @param byte Pointer where the byte is stored
 * @return 1 if a byte was read, 0 if nothing was waiting
 */
uint8_t usb_read(uint8_t *byte);

/**
 * @brief Reads one full line sent by the PC (ends with \n or \r\n)
 *
 * Useful for text commands like "start" or "speed 50". The line is
 * stored in buf without the line ending and is null terminated. Lines
 * longer than size - 1 are cut off. Returns 0 until a full line has
 * arrived, so it can be polled in the main loop.
 *
 * @param buf  Buffer where the line is stored
 * @param size Size of buf in bytes
 * @return 1 if a line was read, 0 if no full line is available yet
 */
uint8_t usb_read_line(char *buf, uint16_t size);

/**
 * @brief Called by CDC_Receive_FS when data arrives from the PC
 *
 * Do not call this yourself. It runs in the USB interrupt and just
 * copies the bytes into the receive buffer.
 *
 * @param buf Received bytes
 * @param len Number of received bytes
 */
void usb_rx_callback(uint8_t *buf, uint32_t len);

#endif /* USB_COMM_H */
