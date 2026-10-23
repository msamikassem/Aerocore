#include "main.h"

uint8_t print_line = 0;         // which line of the debug block is printed next */
Attitude att;
Control ctrl;

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
	    float rates[3] = {0};
	    float dt;

	    float roll_deg = 0.0f;
	    float pitch_deg = 0.0f;

	    uint16_t rc[16] = {0};
	    float motor[4] = {0};

	    uint32_t last_print = 0;

	    aerocore_init();


    while (1)
    {
        // Is the radio alive?
        uint8_t rc_ok = rc_seen && ((HAL_GetTick() - rc_last_tick) < RC_TIMEOUT_MS);

        if (imu_ready)
        {
            __disable_irq();
            imu_ready = 0;
            dt = (float)imu_cycles / CPU_HZ;
            for (int i = 0; i < 3; i++)
            {
                gyro[i]  = imu_gyro[i] * DEG2RAD;
                accel[i] = imu_accel[i];
            }
            __enable_irq();

            // Skip a sample with a bad dt instead of feeding it to the filter
            if (dt > 0.0f && dt < 0.1f)
            {
                // 1. Attitude
                attitude_update(&att, gyro, accel, dt);
                attitude_get_rates(&att, gyro, rates);

                roll_deg  = attitude_get_roll(&att)  * RAD2DEG;
                pitch_deg = attitude_get_pitch(&att) * RAD2DEG;

                // 2. Control
                if (rc_ok)
                {
                    ControlInput in;
                    in.roll_rad    = attitude_get_roll(&att);
                    in.pitch_rad   = attitude_get_pitch(&att);
                    in.rate_rad[0] = rates[0];
                    in.rate_rad[1] = rates[1];
                    in.rate_rad[2] = rates[2];
                    in.rc          = rc;

                    control_update(&ctrl, &in, dt, motor);
                }
                else
                {
                    // Radio lost: disarm and stop the motors */
                    ctrl.armed = 0;
                    for (int i = 0; i < 4; i++)
                    {
                        motor[i] = 0.0f;
                    }
                }

                // 3. Change PWM
                for (int i = 0; i < 4; i++)
                {
                    change_pwm(i + 1, motor[i]);
                }
            }
        }

        // Radio: runs once per RC packet
        if (rc_ready)
        {
            __disable_irq();
            rc_ready = 0;
            CRSF_get_channels(rc);
            __enable_irq();
        }


        //Debug print: only when the USB link is free

#if DEBUG_PRINT
        if ((HAL_GetTick() - last_print) >= PRINT_PERIOD_MS && usb_tx_ready())
        {
            last_print = HAL_GetTick();

            // Values used by the lines below

            const char *arm_text = "DISARMED";

            if (ctrl.armed)
            {
                arm_text = "ARMED   ";
            }

            const char *mode_text = "ANGLE";

            if (rc[CH_MODE - 1] > SWITCH_HIGH)
            {
                mode_text = "ACRO ";
            }

            const char *link_text = "LOST";

            if (rc_ok)
            {
                link_text = "OK  ";
            }

            // Throttle stick in percent
            int thr_percent = (int)((rc[CH_THROTTLE - 1] - CRSF_MIN) * 100.0f / (CRSF_MAX - CRSF_MIN));

            if (thr_percent < 0)
            {
                thr_percent = 0;
            }

            if (thr_percent > 100)
            {
                thr_percent = 100;
            }

            // Print ONE line (or pair of lines) per pass, then move on to the next

            switch (print_line)
            {
                case 0:
                    usb_printf("%s | mode %s | radio %s | thr %3d%%\r\n",
                               arm_text, mode_text, link_text, thr_percent);
                    break;

                case 1:
                    usb_printf("roll %6.1f  pitch %6.1f | gx %5.0f  gy %5.0f  gz %5.0f dps\r\n",
                               roll_deg, pitch_deg,
                               rates[0] * RAD2DEG, rates[1] * RAD2DEG, rates[2] * RAD2DEG);
                    break;
                case 2:
                	usb_printf("ax %6.3f  ay %6.3f  az %6.3f g\r\n",
                	               accel[0], accel[1], accel[2]);
                	break;


            }

            print_line++;

            if (print_line >= 3)
            {
                print_line = 0;
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
            // Stop if the IMU fails
        }
    }

    // Initialize attitude estimator
    attitude_init(&att, stored_gyro_offset);
    attitude_set_level_trim(&att, -1.0f, 0.5f);

    // Initialize controller
    control_init(&ctrl);
}
