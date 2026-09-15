# Firmware

- Not started yet, the hardware needs to come back from JLCPCB and go through testing first (power rails, IMU communication over SPI) before it makes sense to write flight code against it.

- Might start writing some useful code that I can use later on (SPI communication, UART and so on)

## Planned approach

- Bare-metal , mainly so I actually understand what's happening at the register level rather than dropping straight into an existing flight stack.
- BMI270 driver over SPI1, reading gyro + accel via the interrupt pins (PA1, PB0).
- ADM to transfer measured data to RAM.
- standard PWM output on TIM4 for the 4 ESCs, probably start with plain PWM since it's simpler to get right first, move to DShot later.
- USART2 for ELRS receiver input.
- USB-C for serial output, need to remember this board has no VBUS sensing wired up, so USB init needs to explicitly disable VBUS detection rather than wait on it.
- ST-LINK for configuration/flashing
- Eventually: basic rate/angle mode flight control loop, once sensor fusion and motor output are both confirmed working independently.

## Tasks

**SPI Configuration**
[] Configure SPI
[] Write Read Functions
[] integrate ADM
