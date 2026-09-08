# Technical Specs

Reference doc for the actual pin mapping, power tree, and stackup. Written mainly so I don't have to reopen KiCad every time I forget which pin does what.

## MCU pin mapping

| Function | Net | STM32F411 pin |
|---|---|---|
| SPI1 CS (IMU) | SPI1_CS | PA4 |
| SPI1 SCK (IMU) | SPI1_SCK | PA5 |
| SPI1 MISO (IMU) | SPI1_MISO | PA6 |
| SPI1 MOSI (IMU) | SPI1_MOSI | PA7 |
| IMU interrupt 1 | IMU_INT1 | PA1 |
| IMU interrupt 2 | IMU_INT2 | PB0 |
| HSE crystal in | HSE_IN | PH0 |
| HSE crystal out | HSE_OUT | PH1 |
| USB D- | USB_D- | PA11 |
| USB D+ | USB_D+ | PA12 |
| USART2 TX (ELRS) | USART2_TX | PA2 |
| USART2 RX (ELRS) | USART2_RX | PA3 |
| Motor 1 PWM | TIM4_CH1 | PB6 |
| Motor 2 PWM | TIM4_CH2 | PB7 |
| Motor 3 PWM | TIM4_CH3 | PB8 |
| Motor 4 PWM | TIM4_CH4 | PB9 |
| SWCLK (debug) | SWCLK | PA14 |
| SWDIO (debug) | SWDIO | PA13 |
| NRST | NRST | NRST pin, pulled up via 10k, switchable to GND for bootloader entry |
| BOOT0 | BOOT0 | Selected via SPDT switch, GND or 3.3V |
| Status LEDs | — | PB1, PB2, PB3, PB4 (one per LED, each with its own series resistor) |

Everything else (PC13/14/15, PA0, PA8-10, PA15, PB5, PB10, PB12-15) is currently unused and marked no-connect — left available for future expansion (e.g. a second UART, extra PWM outputs, etc.).

## Power tree
4S Lipo (battery input)
    -> ESC power pads x4
    -> LMR51430 > (5V) > TLV75533 > (3.3V) > MCU, IMU logic


- Buck converter feedback divider: R3 (100k, top) / R4 (13.7k, bottom), targets 4.98V output (0.6V reference × (100+13.7)/13.7).
- LDO input/output decoupling: 1µF ceramic on each side, per datasheet recommendation.
- USB VBUS is intentionally **not connected** to anything on the board, this avoids any interaction between USB host power and the battery rail when both happen to be connected at once. 

## Connectors

| Ref | Function | Pins |
|---|---|---|
| J2 | Battery input | +BATT, GND |
| J4, J6, J8, J10 | ESC power (×4) | +BATT, GND |
| J1, J5, J7, J9, J11 | ESC PWM signal (×4, one spare) | TIM4_CHx, GND |
| J12 | ELRS receiver (UART) | GND, +5V, USART2_TX, USART2_RX |
| J13 | Auxiliary solder pad header | — |
| J3 | USB-C | Full USB2.0 pinout, CC1/CC2 pulled down (5.1kΩ-class) for UFP detection, VBUS/SBU/shield unconnected |
| ST-Link header | Programming/debug | GND, NRST, SWCLK, SWDIO, 3.3V |

All the ESC/battery connectors (everything except J3) are bare copper pads meant for hand-soldering wires directly — not off-the-shelf connector parts, which is why they don't appear in the assembly BOM.

## PCB stackup

4-layer board, JLCPCB **JLC04161H-7628** profile, 1.6mm total thickness.

| Layer | Function | Copper |
|---|---|---|
| L1 (Top / F.Cu) | Signal + power | 1oz |
| L2 (In1.Cu) | Ground plane | 0.5oz |
| L3 (In2.Cu) | Signal + power | 0.5oz |
| L4 (Bottom / B.Cu) | Mixed: power stage components + GND pours | 1oz |

Board outline: 36×36mm. Mounting holes: 4× 2.7mm diameter (clearance for M2.5 screws), spaced 26.5×26.5mm

USB D+/D- differential pair: 0.2mm trace width, 0.45mm gap, routed on L1 referencing the L2 ground plane, targeting 90Ω differential impedance (validated against the real stackup dielectric values using JLCPCB's own impedance calculator, not a generic formula).

## Manufacturing settings used (JLCPCB)

- 4 layers, 1.6mm thickness, 1oz outer / 0.5oz inner copper
- Stackup: JLC04161H-7628
- Surface finish: chosen based on assembly method (HASL fine for machine-assembled fine-pitch parts with a proper stencil; would lean ENIG if hand-soldering)
- Assembly: Top
