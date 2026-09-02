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

## Key specs

| | |
|---|---|
| MCU | STM32F411CEU6, 100MHz Cortex-M4, QFN48 |
| IMU | Bosch BMI270, SPI |
| Input power | 4S LiPo (up to ~60A absolute max draw, 15A per motor) |
| Onboard rails | 5V (LMR51430 buck) → 3.3V (TLV75533 LDO) |
| USB | USB-C, USB2.0 full-speed |
| Board | 4 layers, 36×36mm, 1.6mm thick |
| Stackup | JLC04161H-7628 (1oz outer / 0.5oz inner copper) |
| Mounting holes | 4× M2.5, 26.5×26.5mm spacing (non-standard — see notes) |

## Repo layout

```
hardware/                           -> PCB files
    kicad/                          -> the actual schematic + PCB source files (final version)
    Component/      
        Data_sheets/                -> Data sheets of chosen components 
        My_FootPrints.pretty/       -> FootPrints not avaliable on KiCAD
        Schematics/                 -> Schematics not avaliable on KiCAD
        
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

## Tools used

- KiCad 9 for schematic + PCB

## License

MIT — see [LICENSE](LICENSE). Feel free to use any of this for your own projects; if you spot a mistake I haven't caught yet, I'd genuinely like to know.

## Author

Mohammed Kassem — built as a personal project. [msamikassem@gmail.com/ linkedin.com/in/msamikassem]
