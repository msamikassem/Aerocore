/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file main.h
 * @brief Aerocore main application header
 *
 * This file includes all the modules of the flight controller and
 * contains the unit conversion constants and the USB debug print
 * settings used by main.c.
 *
 * @author Mohammed Kassem
 * @date 10/26/2026
 *
 */

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
#include "manual_tune.h"

#define DEG2RAD         0.01745329252f
#define RAD2DEG         57.2957795131f

#define DEBUG_PRINT     1           // 1 = print over USB, 0 = off
#define PRINT_PERIOD_MS 10U



void Error_Handler(void);

/**
 * @brief Wakes the flight task
 *
 * Call this from the IMU interrupt (EXTI1) after the new sample has
 * been stored and imu_ready has been set. It sets a thread flag that
 * wakes flight_task. If the RTOS kernel is not running yet, it does
 * nothing, because the IMU interrupt is enabled before the kernel
 * starts.
 */
void flight_notify_from_isr(void);


#endif /* __MAIN_H */
