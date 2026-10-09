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

**Update (10/09/2026):**

* I could not find that setting in my CubeIDE version.
* Instead I added `__asm__(".global _printf_float");` at the top of `usb_comm.c`. This forces the linker to include the float part of printf, the same as the `-u _printf_float` linker flag.
* `usb_printf("%.2f")` now prints correctly.

**What I learned:**

* Printing floats is not free on small microcontrollers.
* The same linker flag can be set from code if the IDE menu is not where the guide says.

## Interrupt-driven Sampling

### 6. Getting an exact dt for the EKF

**Date:** 10/09/2026

**Problem:**

* The EKF integrates `rate * dt`, so dt must be the real time between two IMU samples.
* Reading the IMU in a loop gives a dt that depends on how long the loop takes.

**Fix:**

* Set the BMI270 INT1 pin as a data-ready output and connected it to PA1 (EXTI1, rising edge).
* The interrupt handler `EXTI1_IRQHandler` (in `bmi.c`) reads the accelerometer and gyroscope and sets `imu_ready = 1`.
* It also stores the number of CPU cycles since the previous sample in `imu_cycles` (from the DWT cycle counter) and then resets the counter. `dt = imu_cycles / 96 MHz`.
* `dwt_init()` lives in `clk_speed.c` and must be called before `bmi_init()`.
* The main loop checks `imu_ready`, copies the data with interrupts briefly disabled, and runs the EKF once per sample.
* Changed the gyroscope output rate from 3200 Hz to 1600 Hz so every interrupt brings fresh data from both sensors.

**Result:**

* Timestamps from `HAL_GetTick()` confirmed one sample about every 0.625 ms (1600 Hz).

**What I learned:**

* An interrupt handler cannot return a value, so it writes to `volatile` globals and a flag, and the main loop polls the flag.
* `volatile` makes sure the main loop really re-reads the flag from memory instead of using an old copy.
* A disabled interrupt is delayed, not lost, and the cycle counter keeps counting, so the delay does not break dt.

### 7. First dt was huge, and duplicate definitions

**Date:** 10/09/2026

**Problem:**

* The first interval included the whole `bmi_init()` (config upload and delays, about 0.2 s), because the counter started in `dwt_init()`.
* If INT1 is already high when the interrupt is enabled, no rising edge is seen and no interrupt ever fires.
* The build failed with "redefinition of imu_ready": an old copy of the variables and handler was still in `bmi.c`.

**Fix:**

* At the end of `bmi_int_init()`: read accel and gyro once to clear waiting data, clear `EXTI->PR`, enable the interrupt, then set `DWT->CYCCNT = 0`.
* Used a `DT_MAX` check (skip any sample with dt above 0.1 s) while testing. The final `main.c` does not need it, because the counter is reset at the end of `bmi_int_init()`.
* Deleted the old definitions so each variable and the handler exist only once.

**What I learned:**

* An edge-triggered interrupt needs the line to go low first.
* Search the project for a name when the compiler reports a redefinition.

## Attitude EKF

### 8. Roll read about 180 degrees when level

**Date:** 10/09/2026

**Problem:**

* The EKF assumes Z points down, so a level board reads az = -1 g.
* The BMI270 reads az = +1 g when flat, so roll came out near 178 degrees.

**Fix:**

* Changed `imu_to_body()` to flip Y and Z (a 180 degree rotation about X). It is applied to both the gyro and the accelerometer.

**Result:**

* Level now reads roll about -1.5 degrees, pitch about 1 degree.

**What I learned:**

* Flipping only one axis makes a mirror image (left-handed frame). Flip two axes to get a real rotation.
* Check the direction with tilt tests (right side down gives positive roll, nose up gives positive pitch).

### 9. Small angle offset at rest

**Date:** 10/09/2026

**Problem:**

* A level board still read about 1 degree on both axes, and the value changed with the surface it sat on (-1.47 / 0.99 on one, about -1.0 / 0.5 on another).

**Cause:**

* Mounting error of the flight controller, a small accelerometer zero-g offset and surfaces that are not perfectly level.

**Fix:**

* Added `attitude_set_level_trim(att, roll_deg, pitch_deg)`. The trim is subtracted from the accelerometer angles inside `correct()`, so the filter state itself is level-referenced.
* The two values are measured once while the drone is on a surface I trust, and must be redone after remounting the board.

**What I learned:**

* A 0.5 to 1.5 degree static error is normal, and a flight controller removes it with a level calibration.

### 10. Shaking the drone corrupts the angle

**Date:** 10/09/2026

**Problem:**

* Shaking the drone on the ground, mostly along Y, moved the estimated roll a lot (about -18 degrees with the first version).

**Cause:**

* The accelerometer cannot tell a sideways push from a tilt. During the shaking `acc_roll` swung between about 12 and 49 degrees while the board stayed on the ground.
* Checked the gyro: at rest it is within about 0.3 dps with no steady bias, so the stored offsets are fine, and the gyro was not the cause.

**Fix:**

* The measurement noise R is now scaled by `r_scale = 1 + e^2 + ie^2`, capped at 50:
  * `e` grows when the accelerometer magnitude is not 1 g.
  * `ie` grows when the accelerometer angle disagrees with the filter angle.
* Base R raised from 3e-3 to 3e-2.
* Gyroscope range changed from 250 dps to 2000 dps, so fast rotations do not saturate the sensor.

**Result:**

* The worst shake test dropped from -26 degrees to about 7 degrees on roll, with pitch under 1 degree.
* A timestamped capture (1 ms resolution) showed roll swinging about +-8 degrees during the shaking, and the recovery after stopping looked to be under a second.

**What I learned:**

* The first attempt (only checking the magnitude) did nothing for sideways pushes, since they barely change the total magnitude, and one version made the result worse (-26 degrees).
* More distrust of the accelerometer means less error while shaking but slower recovery afterwards. It is a trade-off to tune.
* Do not assume the print rate. I misjudged the recovery time because I guessed how often lines were printed. Print a timestamp with the data.
* Hand shaking on the ground is harsher than flight. For real flight, motor vibration will be the bigger accelerometer problem.

## Radio Receiver (UART + CRSF)

### 11. Radio channels printed nothing once the EKF was in the program

**Date:** 10/09/2026

**Problem:**

* An older program with only the radio worked (UART + CRSF, 16 MHz clock).
* After combining it with the EKF and the 96 MHz clock, nothing printed.

**Cause (several faults together):**

* `UART2_init()` was never called. The line `void UART2_init(void);` inside `main()` is only a declaration.
* The baud rate was calculated for 16 MHz. After `clock_96MHz()`, USART2 runs from APB1 at 48 MHz, so the real baud rate was about three times too high.
* The init order was wrong: the UART must be set up after the clock is final.
* The main loop read one UART byte per IMU sample (1600 per second), but about 6500 bytes per second arrive (one every 24 us). The USART holds one byte, so the others were overwritten, and a CRSF packet needs all of its bytes in order.
* `UART2_read()` blocks until a byte arrives, which also stalled the EKF.
* Once the receive interrupt was on, no function named `USART2_IRQHandler` existed in the project, so the first byte would hang the program in the default handler. After adding it, `UART2_read()` could not be used at the same time, because the handler empties the data register first.

**Fix:**

* Removed the stray declarations and called `UART2_init()` after `usb_init()`.
* Set `SYS_CLK` in `uart.c` to 48 MHz, which gives BRR = 114 (about 421053 baud).
* Receive bytes in `USART2_IRQHandler`: it feeds each byte to `CRSF_process_byte()` and sets `rc_ready` when a full RC packet has been decoded. The main loop only checks `rc_ready`.
* USART2 interrupt priority 0, IMU interrupt (EXTI1) priority 1, so a radio byte is never lost while the IMU handler reads the sensor over SPI.

**Result:**

* A test program printed the real values: `pclk1 = 48000000`, `brr = 114`.
* About 3270 bytes and 123 RC packets every half second (about 246 packets per second), with the error counter staying at 4 after startup.
* Channels read 992 for the centered sticks and 174 for low throttle.

**What I learned:**

* A function declaration is not a call.
* Peripheral clocks change when the system clock changes. Print the real clock (`HAL_RCC_GetPCLK1Freq()`) instead of assuming it.
* A one-byte receiver needs an interrupt (or DMA) as soon as the loop does anything slow.
* Counters (bytes, errors, packets) printed together show quickly if the fault is wiring, baud rate or parsing.
* Changing several things at once means I cannot say which one fixed it.

### 12. Build errors when moving the handler into uart.c

**Date:** 10/09/2026

**Problem:**

* "implicit declaration of function CRSF_process_byte" in `uart.c`.
* Then "multiple definition of rc_ready" from the linker.

**Cause:**

* `uart.c` did not include `crsf.h`.
* `rc_ready` was defined (not just declared) in `uart.h`, so every file including the header got its own copy.

**Fix:**

* Added `#include "crsf.h"` to `uart.c`.
* The header only has `extern volatile uint8_t rc_ready;`, and the one definition (with the value) is in `uart.c`.

**What I learned:**

* A header declares (`extern`), and exactly one `.c` file defines.

### 13. CRSF channel range is not 0 to 2047

**Date:** 10/09/2026

**Problem:**

* The old throttle mapping used 0 to 2047, but the printed minimum throttle was 174.

**Fix:**

* Clamp the value to 172 to 1811 and map that range to 1000 to 2000 us. Without it, minimum throttle gives about 1085 us instead of 1000 us, so the motors would idle slightly on.
* The min and max are the usual CRSF values. They still have to be checked against my own radio by moving the sticks to the ends.

**What I learned:**

* Read the real numbers from the radio before choosing a mapping.

### 14. Detecting a lost radio link

**Date:** 10/09/2026

**Problem:**

* `rc_ready` is cleared after every packet, so `rc_ready == 0` is the normal state between packets (about 4 ms at 250 Hz). It cannot show a lost link.

**Fix:**

* Store the time of the last decoded RC packet (`rc_last_tick`) and compare it with `HAL_GetTick()`. If the packet is older than a timeout (500 ms to start with), the link is treated as lost and the failsafe applies.

**What I learned:**

* A flag says "something happened". A lost link needs a timestamp and a timeout.
* The timeout only works if the ELRS receiver stops sending RC packets when the link is lost. If it is set to send fixed failsafe values, the packets never stop.

## Checked Results

| Test | Result |
|------|--------|
| CHIP_ID | 0x24 |
| INTERNAL_STATUS after config upload | 1 |
| Accelerometer, board flat | Z about 1006 mg, X about 35 mg, Y about 3 mg |
| Gyroscope, board still | within about +-0.7 dps, average near 0 |
| Sample rate (timestamps) | about 1600 Hz (0.625 ms between samples) |
| Gyroscope calibrated, board still | within about +-0.3 dps on all axes, average near 0 |
| EKF, board level (before trim) | roll about -1.5 deg, pitch about 1 deg |
| EKF, hard shake on ground | roll swing about +-8 deg, pitch under 1 deg |
| USART2 clock / baud rate | PCLK1 = 48 MHz, BRR = 114 (about 421053 baud) |
| Radio receive rate | about 246 RC packets per second, about 6500 bytes per second |
| Radio sticks centered / throttle low | 992 / 174 |

The small accelerometer X offset and the gyroscope noise are normal for an IMU at rest.

## Open Issues / To Do

* The bias states (`x[2]`, `x[3]`) are not subtracted from the gyro in `predict()`, so the filter cannot track gyro drift on its own. Only the startup calibration offsets are used.
* Pitch is wrapped to +-pi, but it should be limited to +-pi/2 (Euler angle singularity at 90 degrees).
* `S = H P H^T + R` is computed as `H P + R`. It works only because H selects the first two states, and it must change if H changes.
* Mount the flight controller on foam or rubber and check the BMI270 filter bandwidth settings, since motor vibration will affect the accelerometer.
* Store the gyro offsets and the level trim in flash so they survive power cycles.
* Add the CRC check to the CRSF parser. Right now a corrupted packet with the right length and type would be decoded.
* Add the radio link timeout (problem 14) to `main.c` and set the ELRS failsafe mode so that RC packets stop when the link is lost.
* Check the real minimum and maximum of every channel on my radio (CRSF_MIN and CRSF_MAX are the usual values).
* Check `pwm.c`: it was written for the 16 MHz clock, and TIM4 sits on APB1, which runs at 48 MHz now, so the ESC pulse widths must be recalculated before connecting a motor.
* Measure the EKF update time and count IMU overruns (the main loop must finish a sample before the next interrupt).
* Build with optimization (-O2 or the Release configuration).
* Test the filter in flight.
