# Firmware

Bare-metal firmware for the Aerocore flight controller using the STM32F411 and BMI270.

## Current Status

* STM32F411 project created.
* 16 MHz external crystal configured.
* PLL configured for 100 MHz system clock.
* APB1 configured to 50 MHz and APB2 to 100 MHz.
* PB3 LED tested at 100 MHz.
* ST-LINK working for flashing and debugging.

## Planned Approach

* Bare-metal C to learn how the STM32 works at the register level.
* SPI1 for the BMI270.
* DMA for transferring data.
* TIM4 PWM for the 4 ESCs.
* USART2 for the ELRS receiver.
* USB-C for serial communication.
* Eventually add sensor fusion and basic flight control.

## Toolchain

* STM32CubeMX — used only to generate the initial project files.
* STM32CubeIDE — used to write, build, and debug the firmware.
* ARM GNU Toolchain — used to compile the firmware.
* ST-LINK — used to flash and debug the PCB.

## Tasks

**Clock**

* [x] Configure 16 MHz HSE
* [x] Configure 100 MHz PLL
* [x] Test clock using PB3 LED

**USB-C**

* [ ] Configure USB
* [ ] Test serial communication

**SPI / BMI270**

* [ ] Configure SPI
* [ ] Write SPI read/write functions
* [ ] Read BMI270 data
* [ ] Add DMA

**ESCs**

* [ ] Configure TIM4 PWM
* [ ] Test all 4 motors
* [ ] Try DShot later

**ELRS**

* [ ] Configure USART2
* [ ] Read CRSF data
* [ ] Decode channels

**Flight Control**

* [ ] Sensor fusion
* [ ] Rate control
* [ ] Angle control
* [ ] Motor mixing

