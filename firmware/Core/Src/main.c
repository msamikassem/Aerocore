/**
 * @file main.c
 * @brief Aerocore main application
 */

#include "main.h"
#include "manual_tune.h"

uint8_t print_line = 0;
uint32_t last_telemetry = 0U;
uint32_t last_pid_text = 0U;

// CRSF telemetry buffers
uint8_t attitude_packet[10];
uint8_t pid_packet[50];
char pid_text[41];

// PID gain currently selected on the radio
pid_sel_t pid_selected_gain = PID_SEL_P;

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

void aerocore_init(void);

int main(void)
{
    // Variables used by the main loop
    float gyro[3];
    float accel[3];
    float rates[3] = {0.0f, 0.0f, 0.0f};
    float dt;

    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;

    uint16_t rc[16] = {0};
    float motor[4] = {0.0f};

    uint32_t last_print = 0U;

    aerocore_init();

    while (1)
    {
        // Check whether the radio link is alive
        uint8_t rc_ok = rc_seen &&
                        ((HAL_GetTick() - rc_last_tick) < RC_TIMEOUT_MS);

        if (imu_ready)
        {
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
            if (dt > 0.0f && dt < 0.1f)
            {
                // 1. Update attitude estimator
                attitude_update(&att, gyro, accel, dt);
                attitude_get_rates(&att, gyro, rates);

                roll_deg = attitude_get_roll(&att) * RAD2DEG;
                pitch_deg = attitude_get_pitch(&att) * RAD2DEG;

                // 2. Update flight controller
                if (rc_ok)
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

                // 4. Send attitude telemetry every 100 ms
                if ((HAL_GetTick() - last_telemetry) >= 100U)
                {
                    last_telemetry = HAL_GetTick();

                    float roll_rad = attitude_get_roll(&att);
                    float pitch_rad = attitude_get_pitch(&att);

                    // Replace with the estimated yaw angle if available
                    float yaw_rad = 0.0f;

                    uint8_t length = CRSF_build_attitude(
                        attitude_packet,
                        roll_rad,
                        pitch_rad,
                        yaw_rad
                    );

                    UART2_send(attitude_packet, length);
                }

                // 5. Send only the selected PID gain every 100 ms
                if (rc_ok && ((HAL_GetTick() - last_pid_text) >= 100U))
                {
                    last_pid_text = HAL_GetTick();

                    // Format only the selected gain as text
                    CRSF_pid_text(
                        pid_text,
                        sizeof(pid_text),
                        pid_selected_gain,
                        &ctrl
                    );

                    // Build the CRSF flight-mode text frame
                    uint8_t length = CRSF_build_fm_frame(
                        pid_packet,
                        pid_text
                    );

                    // Transmit the selected gain
                    UART2_send(pid_packet, length);
                }
            }
        }

        // Update RC channels when a new packet arrives
        if (rc_ready)
        {
            __disable_irq();

            rc_ready = 0U;
            CRSF_get_channels(rc);

            __enable_irq();
        }

        // USB debug output
#if DEBUG_PRINT
        if (((HAL_GetTick() - last_print) >= PRINT_PERIOD_MS) &&
            usb_tx_ready())
        {
            last_print = HAL_GetTick();

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
