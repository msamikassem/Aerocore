/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file pid.h
 * @brief Generic PID controller
 *
 * A PID controller looks at the difference between what we want
 * and what we have (the measurement), and calculates a
 * correction that pushes the measurement toward the setpoint.
 *
 *   error = setpoint - measurement
 *
 *   P (proportional): reacts to the error right now.
 *                     Big error - big correction.
 *   I (integral)    : adds up the error over time.
 *                     Removes a small error that never goes away.
 *   D (derivative)  : reacts to how fast the measurement is changing.
 *                     Acts like a brake and reduces overshoot.
 *
 *   output = Kp * error  +  Ki * (sum of error * dt)  +  Kd * (rate of change)
 *
 * This file only contains the math and has nothing to do with drones.
 * control.c uses three of these (roll, pitch and yaw).
 *
 * @author Mohammed Kassem
 * @date 10/10/2026
 *
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID controller structure
 *
 * Holds the gains, the limits and the values the controller has to
 * remember from one call to the next.
 */
typedef struct
{
    //Gains (the numbers that are tuned)
    float kp;
    float ki;
    float kd;

    //Limits
    float i_limit;          //The integral part is kept between -i_limit and +i_limit
    float out_limit;        //output is kept between -out_limit and +out_limit

    // Derivative filter
    float d_tau;            //Time constant (s) of the derivative low-pass filter

    // Memory (changes while running, do not set by hand) */
    float integral;         //Integral part (already multiplied by ki)
    float prev_measurement; //Measurement from the previous call
    float d_filtered;       //Filtered rate of change of the measurement
    uint8_t first_call;     //1 until the first call has happened

} Pid;

/**
 * @brief Sets up a PID controller
 *
 * @param pid       pointer to the PID structure
 * @param kp        proportional gain
 * @param ki        integral gain
 * @param kd        derivative gain
 * @param i_limit   maximum size of the integral part
 * @param out_limit maximum size of the output
 * @param d_tau     derivative filter time constant in seconds (for example 0.004)
 */
void pid_init(Pid *pid, float kp, float ki, float kd,
              float i_limit, float out_limit, float d_tau);

/**
 * @brief Clears the memory of the controller
 *
 * Sets the integral and the derivative memory back to zero. Call this
 * whenever the controller was not in use (for example while the drone
 * is disarmed), so old values do not cause a jump when it starts again.
 *
 * @param pid pointer to the PID structure
 */
void pid_reset(Pid *pid);

/**
 * @brief Calculates the PID output
 *
 * Call this once every control cycle.
 *
 * The derivative is calculated from the MEASUREMENT and not from the
 * error. This means a sudden change of the setpoint (a fast stick
 * movement) does not create a big spike in the output.
 *
 * @param pid            pointer to the PID structure
 * @param setpoint       the value we want
 * @param measurement    the value we have
 * @param dt             time since the previous call in seconds
 * @param allow_integral 1 = the integral part may grow, 0 = keep it frozen.
 *                       Set it to 0 when the drone is on the ground, so
 *                       the integral does not build up for nothing.
 * @return The correction, between -out_limit and +out_limit
 */
float pid_update(Pid *pid, float setpoint, float measurement,
                 float dt, uint8_t allow_integral);

#endif /* PID_H */
