/**
 * @file bmi.c
 * @brief BMI270 IMU driver for the STM32F411
 *
 * This file implements the SPI driver for the BMI270 accelerometer
 * and gyroscope. SPI1 is used in mode 0 (CPOL = 0, CPHA = 0) at 6 MHz.
 * The BMI270 supports a maximum SPI clock of 10 MHz.
 *
 * Pin schematic:
 * PA5 --> SCK
 * PA6 --> MISO
 * PA7 --> MOSI
 * PA4 --> CS
 *
 * Sensor configuration:
 * Accelerometer: +-4g (8192 LSB/g), output rate 1600 Hz
 * Gyroscope: +-250 dps (131.2 LSB/dps), output rate 3200 Hz
 *
 * The BMI270 sends one dummy byte before the real data during an
 * SPI read. It also needs a configuration file after every power up,
 * otherwise the sensor data stays at zero.
 *
 * @author Mohammed Kassem
 * @date 10/04/2026
 *
 */


#include "stm32f4xx.h"
#include "main.h"
#include "spi.h"
#include "bmi.h"


// SPI settings
#define SYS_CLK             96000000U
#define prescaler           3       // 96 MHz / 16 = 6 MHz
#define CLOCK_IDLE_LOW      true
#define CAPTURE_FIRST_EDGE  true
#define MSB_First           true
#define eight_bit           true

// BMI270 registers
#define REG_CHIP_ID         0x00
#define REG_ACC_X_LSB       0x0C
#define REG_GYR_X_LSB       0x12
#define REG_INTERNAL_STATUS 0x21
#define REG_ACC_CONF        0x40
#define REG_ACC_RANGE       0x41
#define REG_GYR_CONF        0x42
#define REG_GYR_RANGE       0x43
#define REG_INIT_CTRL       0x59
#define REG_INIT_ADDR_0     0x5B
#define REG_INIT_DATA       0x5E
#define REG_PWR_CONF        0x7C
#define REG_PWR_CTRL        0x7D
#define REG_DUMMY           0x7F    // harmless register used to switch to SPI

#define CHIP_ID_VALUE       0x24
#define CONFIG_CHUNK        32U     // bytes sent per config file write

// Sensitivity
#define ACC_LSB_PER_G       8192.0f     // +-4g
#define GYR_LSB_PER_DPS     16.4f      // +-2000 dps

//interrupt
#define REG_INT1_IO_CTRL    0x53
#define REG_INT_MAP_DATA    0x58

volatile uint8_t  imu_ready  = 0;
volatile uint32_t imu_cycles = 0;
volatile float    imu_gyro[3];     // dps
volatile float    imu_accel[3];    // g


void spi_BMI_pin_config(void)
{
    //Pin schematic:
    //  PA5  --> SCK
    //  PA6  --> MISO
    //  PA7  --> MOSI
    //  PA4  --> CS

    //Provide CLK to GPIOA peripheral (through AHB1)
    RCC->AHB1ENR |= (1U<<0);

    //Set PA5, PA6, PA7 to Alternate function mode (10)
    //PA5
    GPIOA->MODER &= ~(3U<<10); //Set to 00 (reset)
    GPIOA->MODER &= ~(1U<<10); //Set bit 10 to 0
    GPIOA->MODER |=  (1U<<11); //Set bit 11 to 1
    //PA6
    GPIOA->MODER &= ~(3U<<12);
    GPIOA->MODER &= ~(1U<<12);
    GPIOA->MODER |=  (1U<<13);
    //PA7
    GPIOA->MODER &= ~(3U<<14);
    GPIOA->MODER &= ~(1U<<14);
    GPIOA->MODER |=  (1U<<15);

    //Set PA4 to output pin (01)
    GPIOA->MODER &= ~(3U<<8);
    GPIOA->MODER |= (1U<<8);
    GPIOA->MODER &= ~(1U<<9);
    // CS inactive
    GPIOA->ODR |= (1U << 4);

    //Set PA4-PA7 to very high speed (sharp edges)
    GPIOA->OSPEEDR |= (3U<<8) | (3U<<10) | (3U<<12) | (3U<<14);

    //set alternate function to SPI AF05 (0101 -> 5) on AFRL
    GPIOA->AFR[0] |= (5U<<20); //PA5
    GPIOA->AFR[0] |= (5U<<24); //PA6
    GPIOA->AFR[0] |= (5U<<28); //PA7
}

void enable_BMI(void)
{
    GPIOA->ODR &= ~(1U<<4);
}

void disable_BMI(void)
{
    GPIOA->ODR |= (1U<<4);
}


uint8_t bmi_read_reg(uint8_t reg)
{
    uint8_t value;

    bmi_read_burst(reg, &value, 1);

    return value;
}

void bmi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = reg & 0x7F;    // bit 7 = 0 -> write
    buf[1] = value;

    enable_BMI();
    spi_transmit(buf, 2);   // address, then data, in one CS low transaction
    disable_BMI();
}

void bmi_read_burst(uint8_t reg, uint8_t *data, uint32_t len)
{
    uint8_t addr = reg | 0x80;  // bit 7 = 1 -> read
    uint8_t dummy;

    enable_BMI();
    spi_transmit(&addr, 1);
    spi_receive(&dummy, 1);     // BMI270 always sends 1 dummy byte first
    spi_receive(data, len);     // real data
    disable_BMI();
}


uint8_t verify_bmi(void)
{
    spi_BMI_pin_config();
    spi_init(prescaler,CLOCK_IDLE_LOW,CAPTURE_FIRST_EDGE,MSB_First,eight_bit);

    // Dummy read: the CS rising edge switches the BMI270 to SPI
    bmi_read_reg(REG_DUMMY);
    HAL_Delay(1);

    // CHIP_ID, expect 0x24
    return bmi_read_reg(REG_CHIP_ID);
}

void bmi_load_config(void)
{
    uint8_t a[3];
    uint8_t reg = REG_INIT_DATA;

    bmi_write_reg(REG_PWR_CONF, 0x00);      // advanced power save OFF
    HAL_Delay(1);
    bmi_write_reg(REG_INIT_CTRL, 0x00);     // start config load

    for (uint32_t i = 0; i < BMI270_CONFIG_SIZE; i += CONFIG_CHUNK)
    {
        // Tell the sensor where this chunk goes (memory address = i / 2)
        a[0] = REG_INIT_ADDR_0;             // INIT_ADDR_0, then INIT_ADDR_1
        a[1] = (i / 2) & 0x0F;
        a[2] = (i / 2) >> 4;
        enable_BMI();
        spi_transmit(a, 3);
        disable_BMI();

        // Send the chunk
        enable_BMI();
        spi_transmit(&reg, 1);
        spi_transmit((uint8_t *)&bmi270_config_file[i], CONFIG_CHUNK);
        disable_BMI();
    }

    bmi_write_reg(REG_INIT_CTRL, 0x01);     // config load finished
    HAL_Delay(150);
}


void bmi_int_init(void)
{
    // Sensor side: INT1 = output, active high, push-pull; data-ready -> INT1
    bmi_write_reg(REG_INT1_IO_CTRL, 0x0A);
    bmi_write_reg(REG_INT_MAP_DATA, 0x04);

    // MCU side: PA1 as input, rising edge interrupt (EXTI1)
    RCC->AHB1ENR |= (1U<<0);                    // GPIOA clock (already on, harmless)
    RCC->APB2ENR |= (1U<<14);                   // SYSCFG clock
    GPIOA->MODER &= ~(3U<<2);                   // PA1 = input (00)

    SYSCFG->EXTICR[0] &= ~(0xFU<<4);            // EXTI1 -> port A (0000)
    EXTI->RTSR |=  (1U<<1);                     // rising edge
    EXTI->IMR  |=  (1U<<1);                     // unmask line 1

    // Read once to clear any data that is already waiting, so INT1 goes low
    int16_t x, y, z;
    bmi_read_accel(&x, &y, &z);
    bmi_read_gyro(&x, &y, &z);

    EXTI->PR = (1U<<1);                         // clear any edge caught meanwhile

    NVIC_EnableIRQ(EXTI1_IRQn);

    DWT->CYCCNT = 0;                            // first interval starts here
}



void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1U<<1))
    {
        EXTI->PR = (1U<<1);

        imu_cycles = DWT->CYCCNT;      // clocks since the last interrupt
        DWT->CYCCNT = 0;               // restart for the next interval

        float a[3], g[3];
        bmi_read_accel_g(&a[0], &a[1], &a[2]);
        bmi_read_gyro_dps(&g[0], &g[1], &g[2]);

        for (int i = 0; i < 3; i++)
        {
            imu_accel[i] = a[i];
            imu_gyro[i]  = g[i];
        }

        imu_ready = 1;
    }
}

uint8_t bmi_init(void)
{
    // 1. Pins + SPI + SPI wake up + check we are talking to a BMI270
    if (verify_bmi() != CHIP_ID_VALUE)
    {
        return 0;
    }

    // 2. Upload the config file and check the sensor accepted it
    bmi_load_config();
    if ((bmi_read_reg(REG_INTERNAL_STATUS) & 0x0F) != 0x01)
    {
        return 0;
    }

    // 3. Settings
    bmi_write_reg(REG_PWR_CONF, 0x00);      // advanced power save OFF
    HAL_Delay(1);
    bmi_write_reg(REG_ACC_CONF, 0xAC);      // 1600 Hz, normal bandwidth, performance mode
    bmi_write_reg(REG_ACC_RANGE, 0x01);     // +-4g
    bmi_write_reg(REG_GYR_CONF, 0xEC);      // 1600 Hz, performance mode
    bmi_write_reg(REG_GYR_RANGE, 0x00);     // +-2000 dp
    HAL_Delay(1);

    // 4. Turn on gyro (bit 1) and accel (bit 2)
    bmi_write_reg(REG_PWR_CTRL, 0x06);
    HAL_Delay(50);

    // 5. Data-ready interrupt
    bmi_int_init();

    return 1;
}


void bmi_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t b[6];

    bmi_read_burst(REG_ACC_X_LSB, b, 6);

    // each axis = LSB first, then MSB
    *x = (int16_t)((b[1] << 8) | b[0]);
    *y = (int16_t)((b[3] << 8) | b[2]);
    *z = (int16_t)((b[5] << 8) | b[4]);
}

void bmi_read_gyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t b[6];

    bmi_read_burst(REG_GYR_X_LSB, b, 6);

    *x = (int16_t)((b[1] << 8) | b[0]);
    *y = (int16_t)((b[3] << 8) | b[2]);
    *z = (int16_t)((b[5] << 8) | b[4]);
}

void bmi_read_accel_g(float *x, float *y, float *z)
{
    int16_t rx, ry, rz;

    bmi_read_accel(&rx, &ry, &rz);

    *x = rx / ACC_LSB_PER_G;
    *y = ry / ACC_LSB_PER_G;
    *z = rz / ACC_LSB_PER_G;
}

void bmi_read_gyro_dps(float *x, float *y, float *z)
{
    int16_t rx, ry, rz;

    bmi_read_gyro(&rx, &ry, &rz);

    *x = rx / GYR_LSB_PER_DPS;
    *y = ry / GYR_LSB_PER_DPS;
    *z = rz / GYR_LSB_PER_DPS;
}
