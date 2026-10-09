/******************************************************************************
 * Copyright (C) 2017 by Alex Fosdick - University of Colorado
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. Users are
 * permitted to modify this and use it to learn about the field of embedded
 * software. Alex Fosdick and the University of Colorado are not liable for any
 * misuse of this material.
 *
 *****************************************************************************/
/**
 * @file attitude_ekf.c
 * @brief Attitude Extended Kalman Filter (EKF) for a drone
 *
 * This program implements a 4-state attitude EKF. The state vector holds
 * the roll angle, pitch angle, roll bias and pitch bias. The prediction
 * step integrates the Euler angle rates computed from the calibrated
 * gyroscope (p, q, r), and the correction step uses the roll and pitch
 * angles computed from the accelerometer. A stationary gyroscope
 * calibration is also provided, and the gyro offsets are initialized
 * from a value already stored in memory (defined in the main program)
 * instead of being set to zero.
 *
 * @author Mohammed Kassem
 * @date 7/02/2026
 *
 */

#include <math.h>
#include <string.h>
#include <stddef.h>
#include "attitude_ekf.h"


/* Utility functions */

static float wrap_pi(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;

    while (angle < -PI)
        angle += 2.0f * PI;

    return angle;
}


/* IMU AXIS MAPPING
 *
 * Change this function if the physical IMU axes do not match
 * the body-frame convention used by the equations.
 *
 * Current convention:
 *
 * X = forward
 * Y = right
 * Z = down
 *
 * If your IMU is already mounted this way, this function
 * simply copies the values.
 */

static void imu_to_body(const float in[3], float out[3])
{
    out[0] = in[0];
    out[1] = -in[1];
    out[2] = -in[2];
}


/* ACCELEROMETER -> ROLL / PITCH
 *
 * Coordinate convention:
 *
 * X = forward
 * Y = right
 * Z = down
 *
 * When level:
 *
 * ax ~= 0
 * ay ~= 0
 * az ~= -1g
 */

static void accel_to_angles(const float accel[3], float *roll, float *pitch)
{
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];

    *roll = atan2f(-ay, -az);

    *pitch = atan2f(ax, sqrtf(ay * ay + az * az));
}


/* INITIALIZE EKF
 *
 * The gyro offsets are loaded from initial_gyro_offset, which is
 * defined in the main program and holds the values already stored
 * in memory. Passing NULL leaves the offsets at zero.
 */

void attitude_init(Attitude *att, const float initial_gyro_offset[3])
{
    memset(att, 0, sizeof(Attitude));

    /*
     * Load the stored gyro offsets (p, q, r)
     */

    if (initial_gyro_offset != NULL)
    {
        for (int i = 0; i < 3; i++)
        {
            att->gyro_offset[i] = initial_gyro_offset[i];
        }
    }

    /*
     * Initial state
     *
     * roll       = 0
     * pitch      = 0
     * roll bias  = 0
     * pitch bias = 0
     */

    /*
     * Initial covariance
     *
     * These describe how uncertain we are about the
     * initial state.
     * We set them high first because at startup we
     * trust the accelerometer more
     * 0.1 is approx 18 degrees
     */

    att->P[S_ROLL][S_ROLL] = 0.1f;

    att->P[S_PITCH][S_PITCH] = 0.1f;

    att->P[S_BIAS_ROLL][S_BIAS_ROLL] = 0.01f;

    att->P[S_BIAS_PITCH][S_BIAS_PITCH] = 0.01f;
}


/* GYROSCOPE CALIBRATION
 *
 * This is NOT part of the EKF state.
 *
 * It measures the fixed sensor offsets of p, q and r while
 * the drone is completely stationary.
 *
 * Example:
 *
 * Raw gyro:
 *
 * p = +0.012 rad/s
 * q = -0.007 rad/s
 * r = +0.004 rad/s
 *
 * Calibration stores those values as offsets.
 *
 * Later:
 *
 * p = p_raw - p_offset
 * q = q_raw - q_offset
 * r = r_raw - r_offset
 */


/* Start a new calibration */

void attitude_calibrate_start(Attitude *att)
{
    for (int i = 0; i < 3; i++)
    {
        att->cal_sum[i] = 0.0f;
    }

    att->cal_count = 0;
}


/* Add one stationary gyro sample */

void attitude_calibrate_add(Attitude *att, const float gyro_raw[3])
{
    float gyro[3];

    /*
     * Convert IMU axes to body axes.
     */

    imu_to_body(gyro_raw, gyro);


    /*
     * Accumulate gyro samples.
     */

    for (int i = 0; i < 3; i++)
    {
        att->cal_sum[i] += gyro[i];
    }

    att->cal_count++;
}


/* Finish calibration */

void attitude_calibrate_finish(Attitude *att)
{
    if (att->cal_count <= 0)
        return;

    for (int i = 0; i < 3; i++)
    {
        att->gyro_offset[i] = att->cal_sum[i] / (float)att->cal_count;
    }
}


/* EKF PREDICTION
 *
 * State:
 *
 * x = [phi, theta, b_phi, b_theta]
 *
 * Gyroscope:
 *
 * p = calibrated gyro X
 * q = calibrated gyro Y
 * r = calibrated gyro Z
 *
 * The roll/pitch bias states are part of the EKF state.
 */

static void predict(Attitude *att, const float gyro_raw[3], float dt)
{
    float gyro[3];

    /*
     * Convert IMU axes to body axes.
     */

    imu_to_body(gyro_raw, gyro);


    /*
     * Remove the fixed sensor calibration offsets.
     *
     * This is different from b_phi and b_theta.
     */

    float p = gyro[0] - att->gyro_offset[0];

    float q = gyro[1] - att->gyro_offset[1];

    float r = gyro[2] - att->gyro_offset[2];


    /* Current attitude */

    float phi = att->x[S_ROLL];

    float theta = att->x[S_PITCH];


    /* Trigonometric terms */

    float sr = sinf(phi);
    float cr = cosf(phi);

    float st = sinf(theta);
    float ct = cosf(theta);

    float tp = st / ct;


    /* Attitude equations
     *
     * phi_dot =
     *
     * p + (q sin(phi) + r cos(phi)) tan(theta)
     *
     * theta_dot =
     *
     * q cos(phi) - r sin(phi)
     *
     * Convert to Euler rates
     */

    float phi_dot = p + (q * sr + r * cr) * tp;

    float theta_dot = q * cr - r * sr;


    /* Bias model
     *
     * b_phi and b_theta are modeled as slowly changing /
     * constant states.
     *
     * Therefore:
     *
     * b_phi_dot   = 0
     * b_theta_dot = 0
     */

    float bias_roll_dot = 0.0f;

    float bias_pitch_dot = 0.0f;


    /*
     * Euler integration
     */

    att->x[S_ROLL] += phi_dot * dt;

    att->x[S_PITCH] += theta_dot * dt;

    att->x[S_BIAS_ROLL] += bias_roll_dot * dt;

    att->x[S_BIAS_PITCH] += bias_pitch_dot * dt;


    /*
     * Keep angles within [-pi, pi].
     */

    att->x[S_ROLL] = wrap_pi(att->x[S_ROLL]);

    att->x[S_PITCH] = wrap_pi(att->x[S_PITCH]);


    /* STATE TRANSITION MATRIX F
     *
     * F = I + (df/dx) * dt
     */

    float F[N_STATES][N_STATES] = {0};


    /*
     * Partial derivatives of phi_dot
     *
     * d(phi_dot)/d(phi)
     */

    float A = (q * cr - r * sr) * tp;


    /*
     * d(phi_dot)/d(theta)
     */

    float B = (q * sr + r * cr) / (ct * ct);


    /*
     * The bias terms from the model.
     *
     * IMPORTANT:
     *
     * These are the roll/pitch bias states, not gyro
     * p/q sensor offsets.
     *
     * If your exact derivation defines b_phi and b_theta
     * differently in the state equations, these entries
     * must follow that derivation.
     */

    float C = -1.0f;

    float D = 0.0f;


    /*
     * Partial derivatives of theta_dot
     *
     * d(theta_dot)/d(phi)
     */

    float E = -q * sr - r * cr;


    /*
     * d(theta_dot)/d(theta)
     */

    float G = 0.0f;


    /*
     * d(theta_dot)/d(biases)
     */

    float H = 0.0f;

    float I = -1.0f;


    /*
     * F MATRIX
     */

    F[S_ROLL][S_ROLL] = 1.0f + A * dt;

    F[S_ROLL][S_PITCH] = B * dt;

    F[S_ROLL][S_BIAS_ROLL] = C * dt;

    F[S_ROLL][S_BIAS_PITCH] = D * dt;


    F[S_PITCH][S_ROLL] = E * dt;

    F[S_PITCH][S_PITCH] = 1.0f + G * dt;

    F[S_PITCH][S_BIAS_ROLL] = H * dt;

    F[S_PITCH][S_BIAS_PITCH] = I * dt;


    F[S_BIAS_ROLL][S_ROLL] = 0.0f;

    F[S_BIAS_ROLL][S_PITCH] = 0.0f;

    F[S_BIAS_ROLL][S_BIAS_ROLL] = 1.0f;

    F[S_BIAS_ROLL][S_BIAS_PITCH] = 0.0f;


    F[S_BIAS_PITCH][S_ROLL] = 0.0f;

    F[S_BIAS_PITCH][S_PITCH] = 0.0f;

    F[S_BIAS_PITCH][S_BIAS_ROLL] = 0.0f;

    F[S_BIAS_PITCH][S_BIAS_PITCH] = 1.0f;


    /*
     * Process noise Q
     * prediction of the bias states is more reliable than the prediction of roll/pitch
     */

    float Q[N_STATES][N_STATES] = {0};


    Q[S_ROLL][S_ROLL] = 1e-5f;

    Q[S_PITCH][S_PITCH] = 1e-5f;

    Q[S_BIAS_ROLL][S_BIAS_ROLL] = 1e-7f;

    Q[S_BIAS_PITCH][S_BIAS_PITCH] = 1e-7f;


    /*
     * P = F P F^T + Q
     */

    float FP[N_STATES][N_STATES] = {0};

    float P_new[N_STATES][N_STATES] = {0};


    /* FP */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < N_STATES; j++)
        {
            for (int k = 0; k < N_STATES; k++)
            {
                FP[i][j] += F[i][k] * att->P[k][j];
            }
        }
    }


    /* FPF^T + Q */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < N_STATES; j++)
        {
            for (int k = 0; k < N_STATES; k++)
            {
                P_new[i][j] += FP[i][k] * F[j][k];
            }

            P_new[i][j] += Q[i][j];
        }
    }


    memcpy(att->P, P_new, sizeof(P_new));
}


/* EKF CORRECTION
 *
 * Measurement:
 *
 * z = [roll_acc, pitch_acc]
 *
 * Measurement model:
 *
 * h(x) =
 *
 * [ phi   ]
 * [ theta ]
 *
 * Therefore:
 *
 * H =
 *
 * [1 0 0 0]
 * [0 1 0 0]
 */

static void correct(Attitude *att, const float accel_raw[3])
{
    float accel[3];

    imu_to_body(accel_raw, accel);


    /*
     * Accelerometer measurement
     */

    float roll_acc;
    float pitch_acc;

    accel_to_angles(accel, &roll_acc, &pitch_acc);

    // Remove the mounting error so level reads 0
     roll_acc  -= att->level_trim[0];
     pitch_acc -= att->level_trim[1];


    /*
     * Measurement vector:
     *
     * z = [roll_acc, pitch_acc]
     */

    float z[2];

    z[0] = roll_acc;
    z[1] = pitch_acc;


    /*
     * Predicted measurement:
     *
     * h(x) = [phi, theta]
     */

    float h[2];

    h[0] = att->x[S_ROLL];

    h[1] = att->x[S_PITCH];


    /*
     * Innovation:
     *
     * y = z - h(x)
     */

    float y[2];

    y[0] = wrap_pi(z[0] - h[0]);

    y[1] = wrap_pi(z[1] - h[1]);


    /*
     * H matrix
     */

    float H[2][N_STATES] =
    {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f}
    };


    /*
     * Measurement noise R
     *
     * If the accelerometer magnitude is not close to 1 g, the drone is
     * accelerating and the accelerometer angles are unreliable, so
     * R is increased (the filter trusts the gyro more).
     */

    // Total acceleration magnitude (should be about 1g when still)
    float norm = sqrtf(accel[0] * accel[0] +
                       accel[1] * accel[1] +
                       accel[2] * accel[2]);

    // Error from expected 1g acceleration
    float e = (norm - 1.0f) / 0.02f;

    // Largest angle error (about 10 degrees = 0.17 rad)
    float ie = fmaxf(fabsf(y[0]), fabsf(y[1])) / 0.17f;

    // Increase distrust when accel data looks wrong
    float r_scale = 1.0f + e * e + ie * ie;

    // Limit maximum distrust
    if (r_scale > 50.0f)
    {
        r_scale = 50.0f;
    }

    // Measurement noise matrix (larger = trust accel less)
    float R[2][2] =
    {
        {3e-2f * r_scale, 0.0f},
        {0.0f,            3e-2f * r_scale}
    };


    /*
     * S = H P H^T + R
     */

    float S[2][2] = {0};

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for (int k = 0; k < N_STATES; k++)
            {
                S[i][j] += H[i][k] * att->P[k][j];
            }

            S[i][j] += R[i][j];
        }
    }


    /*
     * Inverse of 2x2 S
     */

    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];


    if (fabsf(det) < 1e-9f)
        return;


    float invS[2][2];

    invS[0][0] = S[1][1] / det;

    invS[0][1] = -S[0][1] / det;

    invS[1][0] = -S[1][0] / det;

    invS[1][1] = S[0][0] / det;


    /*
     * K = P H^T S^-1
     */

    float PHt[N_STATES][2] = {0};

    float K[N_STATES][2] = {0};


    /* P H^T */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for (int k = 0; k < N_STATES; k++)
            {
                PHt[i][j] += att->P[i][k] * H[j][k];
            }
        }
    }


    /* K */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                K[i][j] += PHt[i][k] * invS[k][j];
            }
        }
    }


    /*
     * State update
     *
     * x = x + K y
     */

    for (int i = 0; i < N_STATES; i++)
    {
        att->x[i] += K[i][0] * y[0] + K[i][1] * y[1];
    }


    att->x[S_ROLL] = wrap_pi(att->x[S_ROLL]);

    att->x[S_PITCH] = wrap_pi(att->x[S_PITCH]);


    /*
     * P = (I - K H) P
     */

    float KH[N_STATES][N_STATES] = {0};

    float I_KH[N_STATES][N_STATES] = {0};

    float P_updated[N_STATES][N_STATES] = {0};


    /* K H */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < N_STATES; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }


    /* I - K H */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < N_STATES; j++)
        {
            if (i == j)
                I_KH[i][j] = 1.0f - KH[i][j];
            else
                I_KH[i][j] = -KH[i][j];
        }
    }


    /* (I-KH)P */

    for (int i = 0; i < N_STATES; i++)
    {
        for (int j = 0; j < N_STATES; j++)
        {
            for (int k = 0; k < N_STATES; k++)
            {
                P_updated[i][j] += I_KH[i][k] * att->P[k][j];
            }
        }
    }


    memcpy(att->P, P_updated, sizeof(P_updated));
}


/* MAIN EKF UPDATE
 *
 * Call this once every IMU cycle.
 *
 * gyro_raw: [p_raw, q_raw, r_raw]
 *
 * accel_raw: [ax, ay, az]
 *
 * dt:
 *     time since previous update in seconds
 */

void attitude_set_level_trim(Attitude *att, float roll_deg, float pitch_deg)
{
    att->level_trim[0] = roll_deg  * (PI / 180.0f);
    att->level_trim[1] = pitch_deg * (PI / 180.0f);
}

void attitude_update(Attitude *att, const float gyro_raw[3],
                     const float accel_raw[3], float dt)
{
    /*
     * 1. Predict using calibrated gyro
     */

    predict(att, gyro_raw, dt);


    /*
     * 2. Correct using accelerometer
     */

    correct(att, accel_raw);
}


/*
 * OUTPUT FUNCTIONS
 */

float attitude_get_roll(const Attitude *att)
{
    return att->x[S_ROLL];
}


float attitude_get_pitch(const Attitude *att)
{
    return att->x[S_PITCH];
}


float attitude_get_roll_bias(const Attitude *att)
{
    return att->x[S_BIAS_ROLL];
}


float attitude_get_pitch_bias(const Attitude *att)
{
    return att->x[S_BIAS_PITCH];
}
