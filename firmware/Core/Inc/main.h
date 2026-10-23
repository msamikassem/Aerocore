#ifndef __MAIN_H
#define __MAIN_H

#include <math.h>
#include "stm32f4xx.h"
#include "bmi.h"
#include "attitude_ekf.h"
#include "clk_speed.h"
#include "usb_comm.h"
#include "uart.h"
#include "crsf.h"
#include "control.h"
#include "tim4_pwm.h"

#define DEG2RAD         0.01745329252f
#define RAD2DEG         57.2957795131f

#define DEBUG_PRINT     1           // 1 = print over USB, 0 = off
#define PRINT_PERIOD_MS 10U



void Error_Handler(void);


#endif /* __MAIN_H */
