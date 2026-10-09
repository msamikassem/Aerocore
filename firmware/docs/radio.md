# Radio Receiver (UART + CRSF)

Design notes for `uart.c` / `uart.h` and `crsf.c` / `crsf.h`.
The code comments say what each line does. This document explains how the pieces fit together and why.

## Overview

```
ELRS receiver --UART--> USART2 --byte--> RX interrupt --> CRSF parser --> 16 channels
                                                        
```

* `uart.c` receives bytes from the receiver (USART2).
* `crsf.c` collects the bytes into packets and unpacks the channel values.
* The main loop reads the channels when a new packet has been decoded.

## Hardware and UART settings

| Item | Value |
|------|-------|
| Receiver output | CRSF over UART (ELRS receiver) |
| Pins | PA2 = USART2_TX, PA3 = USART2_RX (AF7) |
| Frame | 420000 baud, 8 data bits, no parity, 1 stop bit |
| USART2 clock | APB1 = 48 MHz (with the system clock at 96 MHz) |
| BRR | `(48000000 + 420000 / 2) / 420000 = 114` (about 421053 baud) |

The baud rate depends on the clock of the bus the USART is on. USART2 is on APB1, which is not the CPU clock. The value in `uart.c` (`SYS_CLK`) must match the real APB1 clock. It can be checked at run time with `HAL_RCC_GetPCLK1Freq()`. If the clock setup in `clock_96MHz()` changes, update `SYS_CLK`.

`UART2_init()` must be called after `usb_init()` (which sets up the 96 MHz clock).

## CRSF packet format

Every packet:

| Byte | Content |
|------|---------|
| 0 | Address (`CRSF_ADDRESS`, 0xC8 for packets to the flight controller) |
| 1 | Length (type + payload + CRC) |
| 2 | Type |
| 3 .. n-1 | Payload |
| n | CRC |

The total size is `length + 2` bytes. The parser only handles type 0x16 (RC_CHANNELS_PACKED) with a length of 24:

* 22 bytes of payload, holding 16 channels of 11 bits each (176 bits), packed one after the other with no gaps.
* 1 byte of CRC.

Other packet types, such as link statistics, arrive too. They are collected and ignored. That is why the byte count (about 6500 per second) is higher than 246 packets x 26 bytes.

### Channel values

* Each channel is 11 bits (0 to 2047).
* The usual CRSF range is about 172 to 1811, with the center at 992. The values measured on this radio: centered sticks read 992, low throttle reads 174. The exact end points still have to be checked for each channel.
* Channel order is set in the radio. The defaults used here are 1 = roll, 2 = pitch, 3 = throttle, 4 = yaw.

## Parser (`CRSF_process_byte`)

The parser is a small state machine. It receives one byte per call and remembers its state in `static` variables (`packet[]`, `index`, `length`).

1. `index == 0`: wait for the address byte. Anything else is ignored. This is also how the parser re-synchronizes after an error.
2. `index == 1`: read the length. If it is below 2 or above 62, the packet is rejected and the parser starts again.
3. Otherwise, store bytes until `index == length + 2`.
4. If the type is 0x16 and the length is 24, decode the channels and return 1. Any other type is dropped and the parser starts again.

It returns 1 only on the last byte of a good RC packet, so a return value of 1 means "new channel values are available".

### Decoding

For channel `i`, the first bit is at `i * 11`. An 11-bit value can cross into a third byte, so three bytes are read as one 24-bit number, shifted right by `bit_offset` (0 to 7), and masked with `0x07FF`.

## Receiving: interrupt, not polling

A byte arrives every 24 us (10 bits at 420000 baud), and the USART holds only one byte. If a byte is not read before the next one arrives, it is overwritten (an overrun), and a packet needs every byte.

Reading in the main loop does not work once the loop does anything slow, such as the EKF update or the IMU read. So the bytes are received in `USART2_IRQHandler`:

```c
volatile uint8_t rc_ready = 0;       /* defined once, in uart.c */

void USART2_IRQHandler(void)
{
    if (USART2->SR & (USART_SR_RXNE | USART_SR_ORE))
    {
        uint8_t b = (uint8_t)USART2->DR;     /* reading DR clears the flags */

        if (CRSF_process_byte(b))
        {
            rc_ready = 1;
        }
    }
}
```

* The handler is tiny (about 1 to 2 us), so it takes very little CPU time.
* `UART2_read()` (polling) must not be used together with the interrupt, because the handler empties the data register first.
* `rc_ready` is declared `extern` in `uart.h`, so only one file defines it.

### Interrupt priorities

| Interrupt | Priority | Reason |
|-----------|----------|--------|
| USART2 (radio) | 0 | A radio byte must never wait for another handler |
| EXTI1 (IMU data ready) | 1 | The IMU handler reads the sensor over SPI (tens of microseconds), which is longer than one byte time |

With the UART at a higher priority, it can interrupt the IMU handler, so no byte is lost.

## Using the channels in the main loop

```c
if (rc_ready)
{
    __disable_irq();
    rc_ready = 0;
    roll_cmd     = CRSF_get_channel(1);
    pitch_cmd    = CRSF_get_channel(2);
    throttle_cmd = CRSF_get_channel(3);
    yaw_cmd      = CRSF_get_channel(4);
    __enable_irq();
}
```

The interrupts are disabled for the copy so a new packet cannot change the values halfway through.

### Throttle mapping

CRSF 172 to 1811 is clamped and mapped to 1000 to 2000 us:

```
t = clamp(throttle, 172, 1811)
throttle_us = 1000 + (t - 172) * 1000 / (1811 - 172)
```

## Link loss detection

`rc_ready` cannot show a lost link, because it is cleared after every packet and is 0 for most of the time between packets (about 4 ms at 250 Hz).

The approach is a timeout:

* The handler stores the time of the last decoded RC packet: `rc_last_tick = HAL_GetTick()`.
* The main loop treats the link as lost if no RC packet has arrived for more than a timeout (for example 500 ms): `(HAL_GetTick() - rc_last_tick) >= timeout`.
* On a lost link, the failsafe applies (throttle to minimum, motors off).

This only works if the ELRS receiver stops sending RC packets when the link is lost. If it is configured to send fixed failsafe channel values instead, the packets never stop. The failsafe mode must be set accordingly in the ELRS configuration.

## Known limitations

* No CRC check. A corrupted packet with the right length and type would be decoded. The CRSF CRC is a CRC-8 with polynomial 0xD5 over the type and payload.
* Only the RC channels packet is decoded. Link statistics (signal strength, link quality) are ignored.
* Only the receive direction is used. Nothing is sent back to the receiver (no telemetry).
* The value of `CRSF_ADDRESS` (0xC8) is the usual one for this packet direction.
* Failsafe depends on the receiver settings (see above).
* The PWM output code (`pwm.c`) has to be checked against the current clock before connecting an ESC.
