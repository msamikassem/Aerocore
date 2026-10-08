/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file bmi.h
 * @brief BMI270 IMU driver for the STM32F411
 *
 * This file contains the function declarations for the BMI270
 * accelerometer and gyroscope. The sensor is connected to SPI1
 * (PA4 = CS, PA5 = SCK, PA6 = MISO, PA7 = MOSI). The data-ready
 * interrupt pin INT1 is connected to PA1 (EXTI line 1).
 *
 * Accelerometer: +-4g (8192 LSB/g), output rate 1600 Hz
 * Gyroscope: +-250 dps (131.2 LSB/dps), output rate 1600 Hz
 *
 * Every time a new sample is ready, the BMI270 raises INT1. The
 * interrupt handler reads the sensor, stores the data and the number
 * of clock cycles since the previous sample, and sets imu_ready.
 *
 * @author Mohammed Kassem
 * @date 04/10/2026
 *
 */

#ifndef BMI_H
#define BMI_H

#include <stdint.h>

/** Size of the BMI270 configuration file in bytes */
#define BMI270_CONFIG_SIZE  8192U

/** BMI270 configuration file (defined in bmi_config.c) */
extern const uint8_t bmi270_config_file[BMI270_CONFIG_SIZE];

/** Set to 1 by the interrupt when new data is ready. Clear it after use */
extern volatile uint8_t  imu_ready;

/** CPU clock cycles (DWT->CYCCNT) since the previous interrupt */
extern volatile uint32_t imu_cycles;

/** Latest gyroscope data p, q, r in degrees per second */
extern volatile float    imu_gyro[3];

/** Latest accelerometer data ax, ay, az in g */
extern volatile float    imu_accel[3];

/**
 * @brief Sets up the SPI pins and SPI1, then reads the BMI270 CHIP_ID
 *
 * This function configures PA4-PA7, initializes SPI1 and does a
 * dummy read to switch the BMI270 from I2C to SPI mode. It then
 * reads the CHIP_ID register.
 *
 * @return CHIP_ID of the sensor (0x24 for the BMI270)
 */
uint8_t verify_bmi(void);

/**
 * @brief Initializes the BMI270
 *
 * This function checks the CHIP_ID, uploads the configuration file,
 * configures the accelerometer and gyroscope, turns them on and
 * sets up the data-ready interrupt. It must be called once at
 * startup before reading any data. The DWT cycle counter must be
 * started before calling it.
 *
 * @return 1 if the initialization worked, 0 if it failed
 */
uint8_t bmi_init(void);

/**
 * @brief Sets up the data-ready interrupt
 *
 * This function configures the BMI270 INT1 pin (active high,
 * push-pull, data-ready) and sets up PA1 as a rising edge external
 * interrupt (EXTI1). It is called by bmi_init().
 */
void bmi_int_init(void);

/**
 * @brief Interrupt handler for EXTI line 1 (BMI270 INT1)
 *
 * This function runs every time a new sample is ready. It stores
 * the clock cycles since the last interrupt in imu_cycles and
 * resets the cycle counter. It then reads the accelerometer and
 * gyroscope into imu_accel and imu_gyro and sets imu_ready to 1.
 */
void EXTI1_IRQHandler(void);

/**
 * @brief Reads one BMI270 register
 *
 * @param reg Register address
 * @return Value of the register
 */
uint8_t bmi_read_reg(uint8_t reg);

/**
 * @brief Writes one BMI270 register
 *
 * @param reg Register address
 * @param value Value to write
 */
void bmi_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief Reads multiple BMI270 registers in one transaction
 *
 * The dummy byte that the BMI270 sends first during an SPI read
 * is discarded.
 *
 * @param reg Address of the first register
 * @param data Buffer to store the data
 * @param len Number of bytes to read
 */
void bmi_read_burst(uint8_t reg, uint8_t *data, uint32_t len);

/**
 * @brief Uploads the configuration file to the BMI270
 *
 * The BMI270 needs this file after every power up or reset,
 * otherwise the sensor data stays at zero.
 */
void bmi_load_config(void);

/**
 * @brief Reads the raw accelerometer data
 *
 * @param x Raw X value (8192 LSB/g)
 * @param y Raw Y value (8192 LSB/g)
 * @param z Raw Z value (8192 LSB/g)
 */
void bmi_read_accel(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Reads the raw gyroscope data
 *
 * @param x Raw X value (131.2 LSB/dps)
 * @param y Raw Y value (131.2 LSB/dps)
 * @param z Raw Z value (131.2 LSB/dps)
 */
void bmi_read_gyro(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Reads the accelerometer data in g
 *
 * @param x X acceleration in g
 * @param y Y acceleration in g
 * @param z Z acceleration in g
 */
void bmi_read_accel_g(float *x, float *y, float *z);

/**
 * @brief Reads the gyroscope data in degrees per second
 *
 * @param x X angular rate in dps
 * @param y Y angular rate in dps
 * @param z Z angular rate in dps
 */
void bmi_read_gyro_dps(float *x, float *y, float *z);

#endif /* BMI_H */
