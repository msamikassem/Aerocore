# Aerocore — 3" Flight Controller

Aerocore is a flight controller for 3-inch FPV drones, built around an STM32F411 microcontroller and a BMI270 IMU. It handles IMU sensing, ESC motor control, USB configuration, and power regulation from a 4S LiPo on a single 4-layer board.


## What it does

- Runs an STM32F411CEU6 (Cortex-M4, 100MHz) as the main flight computer
- BMI270 6-axis IMU over SPI for gyro/accel
- USB-C mainly for serial communication with a PC
- Powers itself from a 4S LiPo, steps down to 5V (buck) and 3.3V (LDO) on board
- Drives 4 ESCs via PWM 
- UART header for an ELRS receiver
- Dedicated ST-Link header for programming/debugging
- 2 status LEDs + 2 power LEDS (5v and 3v)


## Repo layout

```
hardware/                           -> PCB files
    kicad/                          -> the actual schematic + PCB source files (final version)
firmware/                           -> flight firmware
docs/
```

## Status

- [X] Schematic design
- [ ] PCB layout
- [ ] Ordered from JLCPCB
- [ ] Board assembled and bring-up tested
- [ ] Firmware
- [ ] First flight

## License

MIT — see [LICENSE](LICENSE). Feel free to use any of this for your own projects; if you spot a mistake I haven't caught yet, I'd genuinely like to know.

## Author

Mohammed Kassem — built as a personal project. [msamikassem@gmail.com/ linkedin.com/in/msamikassem]
