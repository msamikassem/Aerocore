/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file control.c
 * @brief Flight controller: sticks + attitude -> four motor values (0 to 1)
 *
 * Implements the controller described in control.h. The output is four
 * motor values between 0.0 and 1.0. Turning them into PWM pulses is not
 * done here.
 *
 * @author Mohammed Kassem
 * @date 10/10/2026
 *
 */

#include "control.h"


/* Keeps a value between a minimum and a maximum. */
static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum){
        return minimum;}

    if (value > maximum){
        return maximum;}

    return value;
}


/* Turns a radio channel value into a number between -1.0 and +1.0.
 *
 *     result = (raw - center) / half range
 *
 * Examples: 992 -> 0.0
 *
 * Anything closer to the center than "deadband" becomes exactly 0 */
static float stick_to_norm(uint16_t raw, float deadband)
{
    float value = ((float)raw - RC_MID) / RC_HALF_RANGE;

    value = clampf(value, -1.0f, 1.0f);

    if (value > -deadband && value < deadband)
    {
        value = 0.0f;
    }

    return value;
}


void control_init(Control *c)
{
    pid_init(&c->roll_rate,  ROLL_PITCH_KP, ROLL_PITCH_KI, ROLL_PITCH_KD,ROLL_PITCH_I_LIMIT, ROLL_PITCH_OUT_LIM, D_FILTER_TAU);

    pid_init(&c->pitch_rate, ROLL_PITCH_KP, ROLL_PITCH_KI, ROLL_PITCH_KD,ROLL_PITCH_I_LIMIT, ROLL_PITCH_OUT_LIM, D_FILTER_TAU);

    pid_init(&c->yaw_rate,   YAW_KP, YAW_KI, YAW_KD,YAW_I_LIMIT, YAW_OUT_LIM, D_FILTER_TAU);

    c->armed = 0;
}


void control_update(Control *c, const ControlInput *in, float dt, float motor[4])
{

     //STEP 1: READ THE STICKS

    float stick_roll  = stick_to_norm(in->rc[CH_ROLL  - 1], RC_DEADBAND);
    float stick_pitch = stick_to_norm(in->rc[CH_PITCH - 1], RC_DEADBAND);
    float stick_yaw   = stick_to_norm(in->rc[CH_YAW   - 1], RC_DEADBAND);

    float throttle = (stick_to_norm(in->rc[CH_THROTTLE - 1], 0.0f) + 1.0f) / 2.0f;
     	  throttle = (float)(in->rc[CH_THROTTLE - 1] - CRSF_MIN) / (CRSF_MAX - CRSF_MIN);


    /*
     * STEP 2: ARMING
     *
     * - Switch ON and the throttle stick is at the bottom -> armed.
     * - Once armed, it stays armed while the switch stays ON (so
     *   raising the throttle does not disarm it).
     */

    if (in->rc[CH_ARM - 1] > SWITCH_HIGH)
    {
        c->armed = 1; //ready to fly
    }
    else if (throttle < THROTTLE_MIN)
    {
        c->armed = 0;
    }


    /*
     * STEP 3: NOT FLYING -> STABILIZATION OFF
     *
     * Disarmed: all motors 0.
     */

    if (!c->armed || throttle < THROTTLE_MIN)
    {
        pid_reset(&c->roll_rate);
        pid_reset(&c->pitch_rate);
        pid_reset(&c->yaw_rate);

        float value;

        if (c->armed == 1)
        {
            value = MOTOR_IDLE;
        }
        else
        {
            value = 0.0f;
        }

        for (int i = 0; i < 4; i++)
        {
            motor[i] = value;
        }

        return;
    }


    /*
     * STEP 4: TARGET ROTATION RATES
     *
     * ACRO mode (mode switch high):
     *   The stick directly sets the rotation speed.
     *
     * ANGLE mode (mode switch low):
     *   The stick sets a tilt angle. We compare it with the real angle
     *   and ask for a rotation speed that grows with the difference:
     *       target rate = ANGLE_KP * (target angle - real angle)
     *
     */

    float target_roll_rate;
    float target_pitch_rate;

    if (in->rc[CH_MODE - 1] > SWITCH_HIGH)
    {
    	//Acro Mode
        target_roll_rate  =  stick_roll  * MAX_ACRO_RATE_RAD;
        target_pitch_rate = -stick_pitch * MAX_ACRO_RATE_RAD;
        /* Drone's nose goes down when pitch is HIGH
         * Therefore a negative sign is needed
         */
    }
    else
    {
    	//Angle mode (Stabilize)
        float target_roll  =  stick_roll  * MAX_ANGLE_RAD;
        float target_pitch = -stick_pitch * MAX_ANGLE_RAD;

        target_roll_rate  = ANGLE_KP * (target_roll  - in->roll_rad);
        target_pitch_rate = ANGLE_KP * (target_pitch - in->pitch_rad);

        target_roll_rate  = clampf(target_roll_rate,  -MAX_RATE_RAD, MAX_RATE_RAD);
        target_pitch_rate = clampf(target_pitch_rate, -MAX_RATE_RAD, MAX_RATE_RAD);
    }

    float target_yaw_rate = stick_yaw * MAX_YAW_RATE_RAD;


    /*
     * STEP 5: RATE PIDs
     *
     * Each PID compares the rotation speed we want with the speed the
     * gyro measures, and returns a correction. The correction is a
     * fraction of the motor range (0.1 = 10% of full power).
     */

    float roll_cmd  = pid_update(&c->roll_rate,  target_roll_rate,  in->rate_rad[0], dt, 1);
    float pitch_cmd = pid_update(&c->pitch_rate, target_pitch_rate, in->rate_rad[1], dt, 1);
    float yaw_cmd   = pid_update(&c->yaw_rate,   target_yaw_rate,   in->rate_rad[2], dt, 1);


    /* ============================================================
     * STEP 6: MOTOR MIXER
     *
     * Every motor starts from the same base power (the throttle)
     *
     * Then each motor adds or subtract the corrections:

     *
     * The average of the four motors stays the same, so the total
     * lift (altitude) does not change. Only the balance changes
     *
     *   Motor            roll   pitch   yaw
     *   M1 rear  right    -      -      +
     *   M2 front right    -      +      -
     *   M3 rear  left     +      -      -
     *   M4 front left     +      +      +
     * ============================================================ */

    float base = MOTOR_IDLE + throttle * (1.0f - MOTOR_IDLE);

    motor[0] = base - roll_cmd - pitch_cmd + yaw_cmd;   // M1 rear right,  CCW
    motor[1] = base - roll_cmd + pitch_cmd - yaw_cmd;   // M2 front right, CW
    motor[2] = base + roll_cmd - pitch_cmd - yaw_cmd;   // M3 rear left,   CW
    motor[3] = base + roll_cmd + pitch_cmd + yaw_cmd;   // M4 front left,  CCW


    /*
     * STEP 7: KEEP THE MOTORS IN RANGE
     *
     * If a motor wants more than 1.0 (full throttle), we lower ALL
     * motors by the same amount, so the different between them (the
     * correction) is kept
     */

    float highest = motor[0];

    for (int i = 1; i < 4; i++)
    {
        if (motor[i] > highest)
            highest = motor[i];
    }

    if (highest > 1.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            motor[i] -= highest - 1.0f;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        motor[i] = clampf(motor[i], MOTOR_IDLE, 1.0f);
    }
}
