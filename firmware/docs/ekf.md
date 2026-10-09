# Attitude EKF

Design notes for the attitude Extended Kalman Filter in `attitude_ekf.c` / `attitude_ekf.h`.
The code comments say what each line does. This document explains why it is built this way.

## Purpose

Estimate the roll and pitch angle of the drone from a BMI270 IMU (gyroscope + accelerometer).

* The **gyroscope** is smooth and fast, but its angle drifts when integrated.
* The **accelerometer** gives an absolute roll/pitch from gravity, but it is noisy and is fooled by any acceleration other than gravity (shaking, pushing, vibration).
* The EKF integrates the gyro (prediction) and uses the accelerometer to slowly correct the drift (correction).

Yaw is not estimated.

## Conventions

| Item | Convention |
|------|------------|
| Body frame | X = forward, Y = right, Z = down |
| Level board | ax ~ 0, ay ~ 0, az ~ -1 g |
| Gyro units (EKF input) | rad/s |
| Accelerometer units (EKF input) | g (only the direction matters, except for the shake check) |
| Angles (EKF internal and output) | radians |
| Positive roll | right side down |
| Positive pitch | nose up |

`imu_to_body()` converts sensor axes to the body frame. The BMI270 reads az = +1 g when flat, so it flips Y and Z (a 180 degree rotation about X). It is applied to both the gyro and the accelerometer. If the board is mounted differently, only this function changes.

The driver outputs the gyro in deg/s. `main.c` multiplies by `DEG2RAD` before calling the filter, and by `RAD2DEG` when printing angles.

## State

```
x = [ phi, theta, b_phi, b_theta ]
```

| Index | Name | Meaning |
|-------|------|---------|
| 0 | `S_ROLL` | roll angle phi |
| 1 | `S_PITCH` | pitch angle theta |
| 2 | `S_BIAS_ROLL` | roll bias b_phi |
| 3 | `S_BIAS_PITCH` | pitch bias b_theta |

The 4x4 covariance `P` describes how uncertain each state is.

## Prediction step

Inputs: gyro p, q, r (rad/s) and dt (s).

1. Subtract the calibration offsets: `p = p_raw - offset[0]`, and the same for q and r.
2. Convert body rates to Euler angle rates:

```
phi_dot   = p + (q sin(phi) + r cos(phi)) tan(theta)
theta_dot = q cos(phi) - r sin(phi)
```

3. Integrate with Euler's method: `x = x + x_dot * dt`. The biases have zero derivative (slowly changing).
4. Wrap the angles to [-pi, pi].
5. Propagate the covariance: `P = F P F^T + Q`.

`F = I + (df/dx) dt`. The non-trivial entries are:

| Entry | Value |
|-------|-------|
| `F[roll][roll]` | `1 + (q cos(phi) - r sin(phi)) tan(theta) dt` |
| `F[roll][pitch]` | `(q sin(phi) + r cos(phi)) / cos^2(theta) dt` |
| `F[roll][bias_roll]` | `-dt` |
| `F[pitch][roll]` | `(-q sin(phi) - r cos(phi)) dt` |
| `F[pitch][pitch]` | `1` |
| `F[pitch][bias_pitch]` | `-dt` |
| bias rows | identity |

## Correction step

The accelerometer gives a direct measurement of the angles:

```
roll_acc  = atan2(-ay, -az)
pitch_acc = atan2(ax, sqrt(ay^2 + az^2))
```

The level trim (see below) is subtracted from both.

* Measurement `z = [roll_acc, pitch_acc]`, model `h(x) = [phi, theta]`, so `H = [[1 0 0 0],[0 1 0 0]]`.
* Innovation `y = z - h(x)`, wrapped to [-pi, pi].
* `S = H P H^T + R`, `K = P H^T S^-1`, `x = x + K y`, `P = (I - K H) P`.
* If `det(S)` is almost zero, the correction is skipped.

## Tuning values and why

| Parameter | Value | Reason |
|-----------|-------|--------|
| Initial P (roll, pitch) | 0.1 | About 18 degrees of uncertainty at startup, so the accelerometer is trusted at first |
| Initial P (biases) | 0.01 | Biases start near zero |
| Q (roll, pitch) | 1e-5 | Gyro integration noise |
| Q (biases) | 1e-7 | Biases change slowly, so they are predicted more reliably than the angles |
| R base | 3e-2 | Accelerometer noise. Raised from 3e-3 after shake tests (see Devlogs, problem 10) |

### Adaptive R (shake rejection)

R is multiplied by `r_scale = 1 + e^2 + ie^2`, capped at 50:

* `e = (|accel| - 1 g) / 0.02` becomes large when the accelerometer does not read 1 g.
* `ie = max(|y_roll|, |y_pitch|) / 0.17` becomes large when the accelerometer angle disagrees with the filter by more than about 10 degrees.

A larger R makes the filter trust the accelerometer less and follow the gyro more. The cap keeps the accelerometer from being ignored completely, so the filter can still recover.

The trade-off: a larger scale gives less error while shaking but slower recovery afterwards. The values 0.02, 0.17 and the cap of 50 are tuning numbers, found by testing, not derived.

## Calibration

### Gyro offsets

The gyro has a small fixed zero offset. It is measured while the drone is completely still:

1. `attitude_calibrate_start()`
2. `attitude_calibrate_add()` for each sample (about 3000 samples, roughly 2 s at 1600 Hz)
3. `attitude_calibrate_finish()` stores the average in `gyro_offset[]`

`attitude_init()` takes the stored offsets (from memory) as an argument, so calibration is optional at startup. The offsets are in rad/s and in the body frame (the calibration applies `imu_to_body()`).

These offsets are separate from the bias states in the EKF. The offsets are a fixed value measured at rest, and the bias states are meant to track slow changes.

### Level trim

A level drone can still read about 0.5 to 1.5 degrees because of mounting error and the accelerometer zero-g offset. `attitude_set_level_trim(att, roll_deg, pitch_deg)` stores the values the filter reads when level, and they are subtracted from the accelerometer angles inside `correct()`. It must be redone if the board is remounted.

## Timing

The filter needs the real time between samples. The BMI270 raises INT1 on every new sample (1600 Hz), and the interrupt handler measures the CPU cycles since the previous sample with the DWT counter: `dt = cycles / 96 MHz`. The first sample after startup is skipped if dt is above 0.1 s.

## Known limitations

* The bias states are not subtracted from the gyro rates in `predict()`, so they do not currently help. Only the calibration offsets are applied. `F` already contains the `-dt` terms for them, so the model and the prediction are not fully consistent.
* Pitch is wrapped to +-pi, but it should stay within +-pi/2. Near 90 degrees pitch, `tan(theta)` and `1/cos^2(theta)` blow up (the Euler angle singularity).
* `S` is computed as `H P + R`, which equals `H P H^T + R` only because H selects the first two states. It must be rewritten if H changes.
* The accelerometer cannot tell a sustained sideways acceleration from a tilt. The adaptive R reduces the effect, but does not remove it.
* Yaw is not estimated.
* Not yet tested in flight.
