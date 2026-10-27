/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file main.c
 * @brief Aerocore main application (CMSIS-RTOS v2)
 *
 * This file starts the hardware and the RTOS and creates three tasks:
 *
 *   flight_task    (highest priority)
 *       Sleeps until the BMI270 interrupt says a new sample is ready.
 *       Then it updates the attitude estimator, runs the flight
 *       controller and writes the four motor PWM values.
 *
 *   telemetry_task (normal priority)
 *       Every 100 ms it sends the attitude and the selected PID gain
 *       to the radio.
 *
 *   debug_task     (lowest priority)
 *       Prints the PID gains over USB.
 *
 * The interrupt handlers (IMU, radio, USB) still work with the global
 * variables from bmi.h and uart.h. The only RTOS part inside an
 * interrupt is flight_notify_from_isr(), which wakes the flight task.
 *
 * @author Mohammed Kassem
 * @date 10/26/2026
 *
 */

#include "main.h"
#include "cmsis_os2.h"

// Stop the motors if no IMU sample arrives for this long
#define IMU_TIMEOUT_MS          200U

// Send telemetry every 100 ms (needs a 1 kHz RTOS tick)
#define TELEMETRY_PERIOD_MS     100U

// Stack sizes in bytes
#define FLIGHT_STACK_BYTES      4096U
#define TELEMETRY_STACK_BYTES   2048U
#define DEBUG_STACK_BYTES       2048U

// Flag the IMU interrupt uses to wake the flight task
#define FLIGHT_FLAG_IMU         0x01U

uint8_t print_line = 0;

// CRSF telemetry buffers
uint8_t attitude_packet[10];
uint8_t pid_packet[50];
char pid_text[41];

// PID gain currently selected on the radio
volatile pid_sel_t pid_selected_gain = PID_SEL_P;

// Latest roll and pitch, shared with the telemetry task
static volatile float telem_roll_rad = 0.0f;
static volatile float telem_pitch_rad = 0.0f;

// Flight controller objects
Attitude att;
Control ctrl;

// Stored gyro offsets
float stored_gyro_offset[3] =
{
    0.00270f,
    -0.00033f,
    -0.00029f
};

// Task handles
osThreadId_t flight_task_id = NULL;
osThreadId_t telemetry_task_id = NULL;
osThreadId_t debug_task_id = NULL;

// Task settings
static const osThreadAttr_t flight_attr =
{
    .name = "flight",
    .priority = (osPriority_t)osPriorityRealtime,
    .stack_size = FLIGHT_STACK_BYTES
};

static const osThreadAttr_t telemetry_attr =
{
    .name = "telemetry",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = TELEMETRY_STACK_BYTES
};

static const osThreadAttr_t debug_attr =
{
    .name = "debug",
    .priority = (osPriority_t)osPriorityLow,
    .stack_size = DEBUG_STACK_BYTES
};

void aerocore_init(void);
static void flight_task(void *arg);
static void telemetry_task(void *arg);
static void debug_task(void *arg);

/* Checks whether the radio link is alive.
 *
 * The link is alive if at least one RC packet has been received and
 * the last one is not older than RC_TIMEOUT_MS.
 */
static uint8_t rc_link_ok(void)
{
    return rc_seen && ((HAL_GetTick() - rc_last_tick) < RC_TIMEOUT_MS);
}

/* Sets all four motors to zero and writes it to the PWM outputs. */
static void motors_stop(float motor[4])
{
    for (int i = 0; i < 4; i++)
    {
        motor[i] = 0.0f;
        change_pwm(i + 1, 0.0f);
    }
}

/* Wakes the flight task. See main.h.
 *
 * The IMU interrupt is enabled inside bmi_init(), which runs before the
 * kernel starts. Calling an RTOS function that early would crash, so
 * nothing happens until the kernel is running.
 */
void flight_notify_from_isr(void)
{
    if ((flight_task_id != NULL) && (osKernelGetState() == osKernelRunning))
    {
        osThreadFlagsSet(flight_task_id, FLIGHT_FLAG_IMU);
    }
}

int main(void)
{
    aerocore_init();

    osKernelInitialize();

    // Create the tasks
    flight_task_id = osThreadNew(flight_task, NULL, &flight_attr);
    telemetry_task_id = osThreadNew(telemetry_task, NULL, &telemetry_attr);
    debug_task_id = osThreadNew(debug_task, NULL, &debug_attr);

    // Stop if a task could not be created (not enough RTOS heap)
    if ((flight_task_id == NULL) ||
        (telemetry_task_id == NULL) ||
        (debug_task_id == NULL))
    {
        Error_Handler();
    }

    // Start the scheduler, this never returns
    osKernelStart();

    while (1)
    {
    }
}

/* Flight task: runs once for every IMU sample.
 *
 *   1. Wait for the IMU interrupt (or stop the motors after a timeout)
 *   2. Copy the RC channels and the IMU data
 *   3. Update the attitude estimator
 *   4. Update the flight controller
 *   5. Write the motor PWM
 */
static void flight_task(void *arg)
{
    (void)arg;

    float gyro[3];
    float accel[3];
    float rates[3] = {0.0f, 0.0f, 0.0f};
    float dt;

    uint16_t rc[16] = {0};
    float motor[4] = {0.0f};

    for (;;)
    {
        // Sleep until the IMU interrupt wakes us
        uint32_t flags = osThreadFlagsWait(
            FLIGHT_FLAG_IMU,
            osFlagsWaitAny,
            IMU_TIMEOUT_MS
        );

        // No IMU sample for too long: disarm and stop the motors
        if (flags & osFlagsError)
        {
            ctrl.armed = 0U;
            motors_stop(motor);

            continue;
        }

        // Update RC channels when a new packet arrives
        if (rc_ready)
        {
            __disable_irq();

            rc_ready = 0U;
            CRSF_get_channels(rc);

            __enable_irq();
        }

        if (!imu_ready)
        {
            continue;
        }

        // Copy the IMU data
        __disable_irq();

        imu_ready = 0U;
        dt = (float)imu_cycles / CPU_HZ;

        for (int i = 0; i < 3; i++)
        {
            gyro[i] = imu_gyro[i] * DEG2RAD;
            accel[i] = imu_accel[i];
        }

        __enable_irq();

        // Skip samples with an invalid time step
        if (!(dt > 0.0f && dt < 0.1f))
        {
            continue;
        }

        // 1. Update attitude estimator
        attitude_update(&att, gyro, accel, dt);
        attitude_get_rates(&att, gyro, rates);

        // Share roll and pitch with the telemetry task
        telem_roll_rad = attitude_get_roll(&att);
        telem_pitch_rad = attitude_get_pitch(&att);

        // 2. Update flight controller
        if (rc_link_ok())
        {
            ControlInput in;

            in.roll_rad = attitude_get_roll(&att);
            in.pitch_rad = attitude_get_pitch(&att);

            in.rate_rad[0] = rates[0];
            in.rate_rad[1] = rates[1];
            in.rate_rad[2] = rates[2];

            in.rc = rc;

            // Update the PID gains and remember the selected gain
            pid_selected_gain = tune_pid(
                rc[9 - 1],    // CH9: tuning enable switch
                rc[7 - 1],    // CH7: select P, I, or D
                rc[10 - 1],   // CH10: tuning slider
                &ctrl
            );

            control_update(&ctrl, &in, dt, motor);
        }
        else
        {
            // Radio lost: disarm and stop the motors
            ctrl.armed = 0U;

            for (int i = 0; i < 4; i++)
            {
                motor[i] = 0.0f;
            }
        }

        // 3. Update motor PWM
        for (int i = 0; i < 4; i++)
        {
            change_pwm(i + 1, motor[i]);
        }
    }
}

/* Telemetry task: sends the attitude and the selected PID gain to the
 * radio every TELEMETRY_PERIOD_MS.
 *
 * osDelayUntil() keeps the period steady, even if sending takes a
 * different time on each run.
 */
static void telemetry_task(void *arg)
{
    (void)arg;

    uint32_t next_wake = osKernelGetTickCount();

    for (;;)
    {
        // Wait for the next 100 ms slot
        next_wake += TELEMETRY_PERIOD_MS;
        osDelayUntil(next_wake);

        // 1. Send attitude telemetry
        // Replace the yaw with the estimated yaw angle if available
        uint8_t length = CRSF_build_attitude(
            attitude_packet,
            telem_roll_rad,
            telem_pitch_rad,
            0.0f
        );

        UART2_send(attitude_packet, length);

        // 2. Send only the selected PID gain
        if (rc_link_ok())
        {
            // Format only the selected gain as text
            CRSF_pid_text(
                pid_text,
                sizeof(pid_text),
                pid_selected_gain,
                &ctrl
            );

            // Build the CRSF flight-mode text frame
            length = CRSF_build_fm_frame(pid_packet, pid_text);

            // Transmit the selected gain
            UART2_send(pid_packet, length);
        }
    }
}

/* Debug task: prints the PID gains over USB every PRINT_PERIOD_MS.
 * It has the lowest priority, so it never delays the flight task.
 */
static void debug_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        osDelay(PRINT_PERIOD_MS);

#if DEBUG_PRINT
        if (usb_tx_ready())
        {
            // Display the current PID gains
            switch (print_line)
            {
                case 0:
                    usb_printf(
                        "P %6.3f  I %6.3f  D %6.5f\r\n",
                        ctrl.pitch_rate.kp,
                        ctrl.pitch_rate.ki,
                        ctrl.pitch_rate.kd
                    );
                    break;

                default:
                    break;
            }

            print_line++;

            if (print_line >= 1U)
            {
                print_line = 0U;
            }
        }
#endif
    }
}

/* Starts all the hardware. This runs before the RTOS kernel starts. */
void aerocore_init(void)
{
    // Initialize hardware
    clock_96MHz();
    usb_init();
    tim4_pwm();
    UART2_init();
    dwt_init();

    // Initialize IMU
    if (!bmi_init())
    {
        while (1)
        {
            // Stop if the IMU initialization fails
        }
    }

    // Initialize attitude estimator
    attitude_init(&att, stored_gyro_offset);
    attitude_set_level_trim(&att, -1.0f, 0.5f);

    // Initialize flight controller
    control_init(&ctrl);
}
