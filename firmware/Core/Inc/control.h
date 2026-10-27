/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file control.h
 * @brief Flight controller: sticks + attitude -> four motor values (0 to 1)
 *
 * This module turns the radio sticks and the attitude estimate (from the
 * EKF) into four motor values between 0.0 (stopped) and 1.0 (full power).
 * Converting those values to PWM pulses is done elsewhere.
 *
 * HOW IT WORKS (one control cycle)
 *
 *   1. Sticks -> targets
 *        ANGLE mode: roll/pitch stick = target tilt angle
 *        ACRO mode : roll/pitch stick = target rotation rate
 *        yaw stick = target yaw rate, throttle stick = base power
 *
 *   2. Angle loop (angle mode only)
 *        target rate = ANGLE_KP * (target angle - real angle)
 *
 *   3. Rate loop (a PID for roll, pitch and yaw)
 *        correction = PID(target rate, rotation rate measured by the gyro)
 *
 *   4. Mixer
 *        motor = throttle +/- roll correction +/- pitch correction +/- yaw correction
 *
 * SIGN CONVENTION (same as the EKF)
 *
 *   X = forward, Y = right, Z = down
 *   positive roll  = right side goes DOWN
 *   positive pitch = nose goes UP
 *   positive yaw   = nose turns RIGHT (clockwise seen from above)
 *
 * MOTOR LAYOUT (quad X, seen from above)
 *
 *          FRONT
 *     M4 (CW)  M2 (CCW)
 *          \    /
 *           \  /
 *           /  \
 *          /    \
 *     M3 (CCW)   M1 (CW)
 *          REAR
 *
 *   M1 = rear right (CW)      M2 = front right (CCW)
 *   M3 = rear left  (CCW)     M4 = front left  (CW)
 *
 * @author Mohammed Kassem
 * @date 10/10/2026
 *
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "pid.h"

//Radio

#define CRSF_MIN  			172.0f	// Min stick
#define CRSF_MAX  			1811.0f	// MAX stick
#define RC_MID              992.0f  //Stick center value
#define RC_HALF_RANGE       820.0f  //Distance from the center to either end
#define RC_DEADBAND         0.02f   //Stick movement below 2% is ignored

#define CH_ROLL             1
#define CH_PITCH            2
#define CH_THROTTLE         3
#define CH_YAW              4
#define CH_ARM              5       //Arm switch. High = armed
#define CH_MODE             6       //Mode switch. Low = angle, high = Acro

#define SWITCH_HIGH         1500U   //A switch channel above this value is "on"

//Flight settings

#define MAX_ANGLE_RAD   	0.9599f    /* 55 degrees */
#define MAX_YAW_RATE_RAD    3.1416f // Full yaw stick = 180 degrees per second
#define MAX_ACRO_RATE_RAD   5.5f    // Acro mode: full stick = about 315 degrees per second
#define MAX_RATE_RAD        5.0f    // Angle loop never asks for more than this rate

#define ANGLE_KP            4.0f    // Angle loop gain. Stabilize rotation speed

#define THROTTLE_MIN        0.05f   // Throttle stick below 5%: motors only idle
#define MOTOR_IDLE          0.05f   // Lowest motor value while armed

// Rate PID gains (starting values, NOT tested)

#define ROLL_PITCH_KP       0.050f
#define ROLL_PITCH_KI       0.35f
#define ROLL_PITCH_KD       0.0007f
#define ROLL_PITCH_I_LIMIT  0.10f
#define ROLL_PITCH_OUT_LIM  0.25f

#define YAW_KP              0.050f
#define YAW_KI              0.35f
#define YAW_KD              0.0f
#define YAW_I_LIMIT         0.10f
#define YAW_OUT_LIM         0.15f

#define D_FILTER_TAU        0.004f  //Derivative low-pass filter time constant (s)

/**
 * @brief Inputs of one control cycle
 */
typedef struct
{
    float roll_rad;         //Roll angle from the EKF (rad)
    float pitch_rad;        //Pitch angle from the EKF (rad)


    //Rotation rates from gyro
    float rate_rad[3];

    const uint16_t *rc;     //The 16 radio channels, rc[0] = channel 1 */

} ControlInput;

/**
 * @brief Flight controller state
 */
typedef struct
{
    Pid roll_rate;
    Pid pitch_rate;
    Pid yaw_rate;

    uint8_t armed;          //1 = motors may spin

} Control;

/**
 * @brief Sets up the flight controller (PID gains, starts disarmed)
 *
 * @param c pointer to the controller structure
 */
void control_init(Control *c);

/**
 * @brief Runs one control cycle
 *
 * Call once after every EKF update.
 *
 * Arming: the drone is armed while the arm switch is ON and the radio
 * is connected, but it only STARTS arming when the throttle stick is at
 * the bottom. It disarms as soon as the switch goes OFF or the radio
 * is lost.
 *
 * @param c      pointer to the controller structure
 * @param in     inputs (angles, rates, radio channels)
 * @param dt     time since the previous call in seconds
 * @param motor  output: 4 motor values, 0.0 to 1.0. [0] = M1 ...
 */
void control_update(Control *c, const ControlInput *in, float dt, float motor[4]);

#endif /* CONTROL_H */
