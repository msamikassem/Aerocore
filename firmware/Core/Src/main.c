#include "main.h"
#include "usb_comm.h"
#include "pid_comm.h"
#include "bmi.h"

uint8_t test;

int main(void)
{
    usb_init();
    HAL_Delay(100);

    uint8_t ok = bmi_init();
    float ax, ay, az, gx, gy, gz;


    while (1)
    {
        if (ok)
        {
        	bmi_read_accel_g(&ax, &ay, &az);
        	bmi_read_gyro_dps(&gx, &gy, &gz);

        	usb_printf("ACC(mg) %5d %5d %5d | GYR(mdps) %6d %6d %6d\r\n",
        	           (int)(ax * 1000), (int)(ay * 1000), (int)(az * 1000),
        	           (int)(gx * 1000), (int)(gy * 1000), (int)(gz * 1000));

        }
        else
        {
            usb_printf("BMI init failed\r\n");
        }
        HAL_Delay(100);
    }
}


