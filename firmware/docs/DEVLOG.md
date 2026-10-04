# Development Log

Problems I ran into while building the Aerocore firmware and how I solved them.

## BMI270 Bring-up

### 1. CHIP_ID always read 0x00

**Date:** 10/04/2026

**Problem:**

* Reading the CHIP_ID register (0x00) over SPI always returned 0x00.
* The expected value for the BMI270 is 0x24.

**Cause:**

* On SPI reads, the BMI270 sends one dummy byte before the real data.
* My code sent the address and then read only one byte, so I was reading the dummy byte.

**Fix:**

* Read two bytes after the address and throw away the first one.
* Wrapped this in a read function so every register read handles the dummy byte the same way.

**Result:** CHIP_ID = 0x24.

**What I learned:**

* Always check the datasheet for how the SPI read works, not just the register address.
* A value of exactly 0x00 can be the dummy byte, not a broken connection.

**Note:** I changed the dummy byte, the SPI clock and the GPIO speed (problems 2 and 3) at the same time, so I did not test which one alone fixed the 0x00. The dummy byte is the one that explains getting exactly 0x00.

### 2. SPI clock was too fast

**Date:** 10/04/2026

**Problem:**

* The prescaler was set to 0, which divides the clock by 2.
* APB2 runs at 96 MHz, so SPI1 ran at 48 MHz.
* The BMI270 supports a maximum of 10 MHz.

**Fix:**

* Changed the prescaler value to 3 (divide by 16), which gives 6 MHz.

**What I learned:**

* The BR bits in SPI_CR1 are not the divider itself: 0 = /2, 1 = /4, 2 = /8, 3 = /16.
* Always compare the SPI clock with the maximum in the sensor datasheet.

### 3. GPIO speed and switching the BMI270 to SPI

**Date:** 10/04/2026

**Problem:**

* The SPI pins were left at the default low output speed, so the clock and data edges were slow.
* The BMI270 starts in I2C mode and only switches to SPI after a rising edge on CS. In my code that edge only happened by accident.

**Fix:**

* Set PA4 to PA7 to very high speed in OSPEEDR.
* Added a dummy read of register 0x7F before the first real read, so CS goes low and then high on purpose.

**What I learned:**

* Output speed matters even for "slow" signals like a few MHz SPI.
* Some sensors choose their interface from what happens on the CS pin after power up.

### 4. Accelerometer and gyroscope data were all zero

**Date:** 10/04/2026

**Problem:**

* Setting the config registers (ACC_CONF, ACC_RANGE, GYR_CONF, GYR_RANGE, PWR_CTRL) worked and they read back correctly.
* The accelerometer and gyroscope values were still all zero.

**Cause:**

* The BMI270 needs an 8 KB configuration file uploaded after every power up or reset.
* Without it the sensor data stays at zero.

**Fix:**

* Added the Bosch configuration file (from the official BMI270 SensorAPI on GitHub) as `bmi_config.c`.
* Wrote `bmi_load_config()` to send it in 32 byte chunks, setting the target address in INIT_ADDR_0 and INIT_ADDR_1 before each chunk.
* Checked INTERNAL_STATUS (0x21): the lower 4 bits read 1 when the upload worked.

**Result:**

* Accelerometer Z read about 1 g (8192 LSB/g) when the board was flat.
* Gyroscope read close to zero when the board was still.

**What I learned:**

* A register reading back correctly only proves the write worked, not that the sensor is running.
* Some sensors need firmware or a config file loaded at startup.

### 5. Printing float values

**Date:** 10/04/2026

**Problem:**

* Printing the converted values (g and dps) with the float format did not work with the default newlib-nano settings.

**Fix:**

* Printed milli-units as integers instead (mg and mdps), by multiplying by 1000 and casting to int.
* The other option is to enable "Use float with printf from newlib-nano" in the project settings, at the cost of a bigger program.

**What I learned:**

* Printing floats is not free on small microcontrollers.

## Checked Results

| Test | Result |
|------|--------|
| CHIP_ID | 0x24 |
| INTERNAL_STATUS after config upload | 1 |
| Accelerometer, board flat | Z about 1006 mg, X about 35 mg, Y about 3 mg |
| Gyroscope, board still | within about ±0.7 dps, average near 0 |

The small accelerometer X offset and the gyroscope noise are normal for an IMU at rest. The plan is to measure and subtract the offsets at startup.
