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
 * @file attitude_ekf.h
 * @brief Attitude Extended Kalman Filter (EKF) for a drone
 *
 * This header declares the interface of a 4-state attitude EKF. The
 * state vector holds the roll angle, pitch angle, roll bias and pitch
 * bias. The filter predicts the attitude using the calibrated gyroscope
 * rates (p, q, r) and corrects it using roll/pitch angles computed from
 * the accelerometer. A stationary gyroscope calibration routine is also
 * provided, and its starting offsets can be loaded from a value that is
 * already stored in memory.
 *
 * @author Mohammed Kassem
 * @date 7/02/2026
 *
 */

#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

#define N_STATES 4

#define S_ROLL        0
#define S_PITCH       1
#define S_BIAS_ROLL   2
#define S_BIAS_PITCH  3

#define PI 3.14159265358979323846f

/**
 * @brief Attitude EKF structure
 *
 * State:
 *   x[0] = roll angle       phi
 *   x[1] = pitch angle      theta
 *   x[2] = roll bias        b_phi
 *   x[3] = pitch bias       b_theta
 */
typedef struct
{
    float x[N_STATES];               /**< State vector */
    float P[N_STATES][N_STATES];     /**< State covariance matrix */

    /* Gyroscope calibration offsets
     *   gyro_offset[0] = p offset
     *   gyro_offset[1] = q offset
     *   gyro_offset[2] = r offset
     */
    float gyro_offset[3];

    /* Level trim: roll/pitch the accelerometer reads when the drone
     * is sitting level (radians). Set with attitude_set_level_trim() */
    float level_trim[2];

    float cal_sum[3];                /**< Calibration accumulator */
    int cal_count;                   /**< Number of calibration samples */

} Attitude;

/**
 * @brief Initializes the EKF
 *
 * Clears the filter structure, sets the initial covariance, and loads
 * the gyroscope offsets (p, q, r) from the array passed in. This array
 * is expected to hold the offsets previously stored in memory and
 * defined in the main program. If a NULL pointer is passed, the offsets
 * are set to zero.
 *
 * @param att pointer to the attitude filter structure
 * @param initial_gyro_offset pointer to an array of 3 floats {p, q, r}
 */
void attitude_init(Attitude *att, const float initial_gyro_offset[3]);

/**
 * @brief Sets the level trim (mounting error of the roll/pitch)
 *
 * Hold the drone level and still, read the roll and pitch the filter
 * reports, and pass those values here. They are subtracted from the
 * accelerometer angles, so a level drone reads 0. Call after
 * attitude_init(). The default is 0.
 *
 * @param att pointer to the attitude filter structure
 * @param roll_deg roll reading when level, in degrees
 * @param pitch_deg pitch reading when level, in degrees
 */
void attitude_set_level_trim(Attitude *att, float roll_deg, float pitch_deg);

/**
 * @brief Starts a new gyroscope calibration
 *
 * Resets the calibration accumulator and sample counter. The drone
 * must be completely stationary during calibration. The currently
 * stored offsets are not changed until attitude_calibrate_finish().
 *
 * @param att pointer to the attitude filter structure
 */
void attitude_calibrate_start(Attitude *att);

/**
 * @brief Adds one stationary gyroscope sample to the calibration
 *
 * Converts the raw gyro sample from IMU axes to body axes and adds
 * it to the accumulator.
 *
 * @param att pointer to the attitude filter structure
 * @param gyro_raw raw gyro sample {p_raw, q_raw, r_raw}
 */
void attitude_calibrate_add(Attitude *att, const float gyro_raw[3]);

/**
 * @brief Finishes the gyroscope calibration
 *
 * Averages the accumulated samples and stores the result as the new
 * gyro offsets. If no samples were added, the offsets are unchanged.
 *
 * @param att pointer to the attitude filter structure
 */
void attitude_calibrate_finish(Attitude *att);

/**
 * @brief Runs one EKF cycle (predict then correct)
 *
 * Call once every IMU cycle. The prediction step uses the gyro
 * (minus the calibration offsets) and the correction step uses the
 * accelerometer.
 *
 * @param att pointer to the attitude filter structure
 * @param gyro_raw raw gyro values {p_raw, q_raw, r_raw} in rad/s
 * @param accel_raw raw accelerometer values {ax, ay, az}
 * @param dt time since the previous update in seconds
 */
void attitude_update(Attitude *att, const float gyro_raw[3],
                     const float accel_raw[3], float dt);

/**
 * @brief Gets the estimated roll angle
 *
 * @param att pointer to the attitude filter structure
 * @return Roll angle in radians
 */
float attitude_get_roll(const Attitude *att);

/**
 * @brief Gets the estimated pitch angle
 *
 * @param att pointer to the attitude filter structure
 * @return Pitch angle in radians
 */
float attitude_get_pitch(const Attitude *att);

/**
 * @brief Gets the estimated roll bias
 *
 * @param att pointer to the attitude filter structure
 * @return Roll bias state
 */
float attitude_get_roll_bias(const Attitude *att);

/**
 * @brief Gets the estimated pitch bias
 *
 * @param att pointer to the attitude filter structure
 * @return Pitch bias state
 */
float attitude_get_pitch_bias(const Attitude *att);

#endif /* ATTITUDE_EKF_H */
