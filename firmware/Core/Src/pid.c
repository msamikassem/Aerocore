/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file pid.c
 * @brief Generic PID controller
 *
 * Implements the PID controller declared in pid.h. See pid.h for an
 * explanation of what P, I and D do.
 *
 * @author Mohammed Kassem
 * @date 10/10/2026
 *
 */

#include "pid.h"


/* Keeps a value between -limit and +limit.
 *
 * Example: clamp(0.5, 0.3) = 0.3
 */
static float clamp_symmetric(float value, float limit)
{
    if (value > limit){
        return limit;}

    if (value < -limit){
        return -limit;}

    return value;
}


void pid_init(Pid *pid, float kp, float ki, float kd, float i_limit, float out_limit, float d_tau)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->i_limit = i_limit;
    pid->out_limit = out_limit;
    pid->d_tau = d_tau;

    pid_reset(pid);
}


void pid_reset(Pid *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered = 0.0f;

    /* The next call to pid_update() is "the first one" again, so the
     * derivative does not use an old measurement. */
    pid->first_call = 1;
}


float pid_update(Pid *pid, float setpoint, float measurement,float dt, uint8_t allow_integral)
{
    // A time step of zero or less is not valid (it would divide by zero in the derivative). Do nothing in that case
    if (dt <= 0.0f)
        return 0.0f;


    /*
     * ERROR: how far are we from where we want to be?
     *
     * Example: we want to rotate at 1.0 rad/s but we rotate at
     * 0.4 rad/s, so error = 0.6 rad/s.
     */

    float error = setpoint - measurement;


    /*
     * P TERM: proportional to the error right now.
     *
     * Big error -> big correction. If kp is too high the drone
     * shakes quickly (oscillates). If it is too low the drone feels
     * soft and slow to react.
     */

    float p_term = pid->kp * error;


    /*
     * I TERM: the sum of the error over time.
     *
     * ki is multiplied BEFORE adding, so changing ki later does not
     * make the integral jump.
     *
     * Two protections:
     * 1. Only grow when allow_integral is 1 (not while on the ground).
     * 2. Keep it between -i_limit and +i_limit ("anti-windup"), so it
     *    cannot grow to a huge value and cause a big overshoot later.
     */

    if (allow_integral)
    {
        pid->integral += pid->ki * error * dt;
        pid->integral = clamp_symmetric(pid->integral, pid->i_limit);
    }


    /*
     * D TERM: how fast the measurement is changing.
     *
     *     (measurement now - measurement before) / dt
     *
     * The minus sign is there because if the measurement is rising
     * quickly, the D term must push against it (a brake)
     *
     * On the very first call there is no "measurement before", so we
     * pretend there was no change
     */

    if (pid->first_call)
    {
        pid->prev_measurement = measurement;
        pid->d_filtered = 0.0f;
        pid->first_call = 0;
    }

    float d_raw = -(measurement - pid->prev_measurement) / dt;

    pid->prev_measurement = measurement;


    /* A derivative makes noise much bigger, because noise changes
     * quickly. A low-pass filter smooths it out
     *
     * alpha is between 0 and 1. A small alpha = a lot of smoothing,
     * a big alpha = little smoothing
     *
     * Each call, the filtered value moves a fraction (alpha) of the
     * way toward the new raw value
     * */

    float alpha = dt / (pid->d_tau + dt);

    pid->d_filtered += alpha * (d_raw - pid->d_filtered);

    float d_term = pid->kd * pid->d_filtered;



     //OUTPUT: add the three parts, then limit the result


    float output = p_term + pid->integral + d_term;

    return clamp_symmetric(output, pid->out_limit);
}
