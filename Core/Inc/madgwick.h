/**
 * madgwick.h  —  Madgwick AHRS / IMU Filter  (STM32F746G-DISCO)
 *
 * Implements the full MARG (Magnetic, Angular Rate, Gravity) variant
 * of the Madgwick gradient-descent orientation filter.
 *
 * Reference:
 *   Madgwick, S.O.H., Harrison, A.J.L., Vaidyanathan, R. (2011).
 *   "Estimation of IMU and MARG orientation using a gradient descent
 *    algorithm." Proc. IEEE Int. Conf. Rehabilitation Robotics (ICORR),
 *   pp. 1-7. doi:10.1109/ICORR.2011.5975346
 *
 * Key advantages over the basic complementary filter:
 *   - Quaternion representation: no gimbal lock, full 3-D orientation
 *   - Gradient descent: optimal correction each step, not just weighted avg
 *   - MARG variant fuses magnetometer → yaw observable (not just roll/pitch)
 *   - β parameter trades gyro integration noise vs accel/mag disturbance
 *   - Computationally efficient: uses fast inverse-square-root (Quake III)
 *
 * Recommended β for ICM-20948 at 100 Hz:
 *   β = sqrt(3/4) × ω_err  where ω_err ≈ gyro_noise × sqrt(ODR)
 *   ω_err ≈ 0.015 dps/√Hz × √100Hz × π/180 ≈ 0.0026 rad/s
 *   β ≈ 0.0026 × 0.866 ≈ 0.041  (use 0.033 conservatively)
 */
#ifndef MADGWICK_H
#define MADGWICK_H

#include <stdint.h>
#include <stdbool.h>

/* ── Filter state ─────────────────────────────────────────────────── */
typedef struct {
    /* Unit quaternion: q = [w, x, y, z]  (Hamilton convention) */
    float q0, q1, q2, q3;   /* w, x, y, z */

    /* Filter gain β — see header comments for tuning guidance */
    float beta;

    /* Derived Euler angles (radians) — updated after each call */
    float roll;   /* rotation about X  (forward axis) */
    float pitch;  /* rotation about Y  (lateral axis)  */
    float yaw;    /* rotation about Z  (vertical axis) — needs mag */

    /* Initialised flag — set true on first valid accel sample */
    bool initialised;
} Madgwick_t;

/* ── API ──────────────────────────────────────────────────────────── */

/**
 * @brief Initialise filter with identity quaternion.
 * @param mw    Filter state struct
 * @param beta  Filter gain (0.033 recommended for ICM-20948 @ 100Hz)
 */
void Madgwick_Init(Madgwick_t *mw, float beta);

/**
 * @brief  Full MARG update — 9-DOF with magnetometer.
 *         Use when AK09916 data is valid (mag DRDY = 1).
 *
 * @param mw          Filter state
 * @param gx,gy,gz    Gyroscope  [rad/s], bias removed
 * @param ax,ay,az    Accelerometer [any unit, will be normalised]
 * @param mx,my,mz    Magnetometer  [any unit, will be normalised]
 * @param dt          Sample period [s]
 */
void Madgwick_UpdateMARG(Madgwick_t *mw,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz,
                         float dt);

/**
 * @brief  IMU-only update — 6-DOF (no magnetometer, yaw drifts).
 *         Use when mag data is stale or disturbed.
 *
 * @param mw          Filter state
 * @param gx,gy,gz    Gyroscope  [rad/s], bias removed
 * @param ax,ay,az    Accelerometer [any unit, will be normalised]
 * @param dt          Sample period [s]
 */
void Madgwick_UpdateIMU(Madgwick_t *mw,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt);

/**
 * @brief  Compute Euler angles (roll/pitch/yaw) from current quaternion.
 *         Results stored in mw->roll, mw->pitch, mw->yaw [radians].
 */
void Madgwick_GetEuler(Madgwick_t *mw);

#endif /* MADGWICK_H */
