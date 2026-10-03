/**
 * @file usb_comm.c
 * @brief Simple send/receive functions for USB CDC (virtual COM port)
 *
 * Receiving: the USB interrupt copies incoming bytes into a ring buffer
 * (usb_rx_callback). The main loop reads them out with usb_read(),
 * usb_read_line() or usb_available().
 *
 * Sending: usb_send() copies the data into an internal buffer and hands
 * it to the CubeMX CDC_Transmit_FS(). It waits for the previous transfer
 * to finish first, with a timeout so it never hangs.
 *
 * @author Mohammed Kassem
 * @date 03/10/2026
 *
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "clk_speed.h"
#include "usb_device.h"
#include "usb_comm.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#define RX_BUF_SIZE      512    // receive ring buffer size in bytes
#define TX_BUF_SIZE      256    // bytes sent per USB transfer
#define TX_TIMEOUT_MS    20     // max wait for a busy transfer

extern USBD_HandleTypeDef hUsbDeviceFS;

// Receive ring buffer: head is written by the USB interrupt,
// tail is read by the main loop
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

// Copy of the data being sent. It must stay valid until the
// transfer is finished, so it can't be a local variable
static uint8_t tx_buf[TX_BUF_SIZE];


void usb_init(void)
{
    // Start HAL (SysTick still runs from the 16 MHz HSI here)
    HAL_Init();

    // Switch to 96 MHz so USB gets exactly 48 MHz
    clock_96MHz();

    // Tell HAL the new clock speed and re-time SysTick (needed for HAL_GetTick)
    SystemCoreClockUpdate();
    HAL_InitTick(TICK_INT_PRIORITY);

    // Start USB (PA11/PA12, clock, interrupt, CDC class)
    MX_USB_DEVICE_Init();
}


// Called by usb_device.c if something fails during USB init.
// Weak, so defining your own Error_Handler elsewhere overrides this one
__attribute__((weak)) void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}


uint8_t usb_connected(void)
{
    // The PC has enumerated the device
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}


// Waits until the previous transfer is done. Returns 1 if the
// link is free, 0 if it timed out
static uint8_t usb_wait_tx_free(void)
{
    uint32_t start = HAL_GetTick();
    USBD_CDC_HandleTypeDef *hcdc;

    while (1)
    {
        hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

        if (hcdc != NULL && hcdc->TxState == 0)
        {
            return 1;
        }
        if ((HAL_GetTick() - start) > TX_TIMEOUT_MS)
        {
            return 0;
        }
    }
}


uint16_t usb_send(const uint8_t *data, uint16_t len)
{
    uint16_t sent = 0;

    // Nobody is listening, don't waste time
    if (!usb_connected())
    {
        return 0;
    }

    while (sent < len)
    {
        uint16_t chunk = len - sent;
        if (chunk > TX_BUF_SIZE)
        {
            chunk = TX_BUF_SIZE;
        }

        // Previous transfer still using tx_buf, wait for it
        if (!usb_wait_tx_free())
        {
            break;
        }

        memcpy(tx_buf, &data[sent], chunk);

        if (CDC_Transmit_FS(tx_buf, chunk) != USBD_OK)
        {
            break;
        }
        sent += chunk;
    }

    return sent;
}


void usb_print(const char *str)
{
    usb_send((const uint8_t *)str, (uint16_t)strlen(str));
}


void usb_printf(const char *fmt, ...)
{
    char text[128];
    va_list args;

    va_start(args, fmt);
    int n = vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    if (n < 0)
    {
        return;
    }
    if (n >= (int)sizeof(text))
    {
        n = sizeof(text) - 1;   // output was cut off
    }

    usb_send((const uint8_t *)text, (uint16_t)n);
}


uint16_t usb_available(void)
{
    return (uint16_t)((rx_head + RX_BUF_SIZE - rx_tail) % RX_BUF_SIZE);
}


uint8_t usb_read(uint8_t *byte)
{
    if (rx_head == rx_tail)
    {
        return 0;
    }

    *byte = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}


uint8_t usb_read_line(char *buf, uint16_t size)
{
    uint16_t avail = usb_available();
    uint16_t count = 0;
    uint16_t pos = rx_tail;
    uint8_t found = 0;

    if (size == 0)
    {
        return 0;
    }

    // Look for a newline without removing anything yet
    while (count < avail)
    {
        if (rx_buf[pos] == '\n')
        {
            found = 1;
            break;
        }
        pos = (pos + 1) % RX_BUF_SIZE;
        count++;
    }

    // Buffer is completely full with no newline: hand it over as a
    // line anyway, otherwise it would block forever
    if (!found && avail < (RX_BUF_SIZE - 1))
    {
        return 0;
    }

    // Copy the line out, skipping \r and \n, cut off if too long
    uint16_t out = 0;
    uint16_t take = found ? (count + 1) : avail;

    for (uint16_t i = 0; i < take; i++)
    {
        uint8_t c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        if (c != '\r' && c != '\n' && out < (size - 1))
        {
            buf[out++] = (char)c;
        }
    }
    buf[out] = '\0';

    return 1;
}


void usb_rx_callback(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;

        // If the buffer is full the new byte is dropped
        if (next != rx_tail)
        {
            rx_buf[rx_head] = buf[i];
            rx_head = next;
        }
    }
}
