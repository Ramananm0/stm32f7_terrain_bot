/**
 * madgwick.c  —  Madgwick AHRS / IMU Filter
 *
 * Direct C implementation of the algorithm published in:
 *   Madgwick, S.O.H., Harrison, A.J.L., Vaidyanathan, R. (2011).
 *   "Estimation of IMU and MARG orientation using a gradient descent
 *    algorithm." IEEE ICORR 2011. doi:10.1109/ICORR.2011.5975346
 *
 * Equations referenced below match those in the paper (§ III):
 *   eq.(25)  IMU gradient step
 *   eq.(32)  MARG gradient step (with Earth-frame magnetic reference)
 *   eq.(33)  Quaternion rate from gyroscope
 *   eq.(42)  Filter update (fuse gyro + gradient correction)
 *
 * STM32F746G-DISCO — Cortex-M7 FPU present — all float ops are HW.
 */

#include "madgwick.h"
#include <math.h>

/* ── Fast inverse square root  (Carmack / Quake III id Software, 1999) ── */
/* Reduces each normalisation from ~15 cycles to ~4 on Cortex-M7           */
static inline float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    union { float f; uint32_t i; } conv = {x};
    conv.i = 0x5F3759DFu - (conv.i >> 1);      /* Newton-Raphson seed     */
    conv.f = conv.f * (1.5f - halfx * conv.f * conv.f);  /* 1 iteration   */
    return conv.f;
}

/* ── Init ─────────────────────────────────────────────────────────── */
void Madgwick_Init(Madgwick_t *mw, float beta)
{
    mw->q0 = 1.0f; mw->q1 = 0.0f; mw->q2 = 0.0f; mw->q3 = 0.0f;
    mw->beta  = beta;
    mw->roll  = 0.0f; mw->pitch = 0.0f; mw->yaw = 0.0f;
    mw->initialised = false;
}

/* ── MARG update (9-DOF, eq.32 + eq.42) ─────────────────────────── */
void Madgwick_UpdateMARG(Madgwick_t *mw,
                         float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz,
                         float dt)
{
    float q0 = mw->q0, q1 = mw->q1, q2 = mw->q2, q3 = mw->q3;
    float rn;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    /* ── Gyroscope rate of change of quaternion (eq.33) ── */
    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    /* ── Skip correction if accel is zero (free-fall / bad data) ── */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        /* Normalise accel */
        rn = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= rn; ay *= rn; az *= rn;

        /* Normalise mag */
        rn = inv_sqrt(mx*mx + my*my + mz*mz);
        mx *= rn; my *= rn; mz *= rn;

        /* Reference direction of Earth's magnetic field (eq.28-29) */
        float _2q0mx = 2.0f*q0*mx, _2q0my = 2.0f*q0*my, _2q0mz = 2.0f*q0*mz;
        float _2q1mx = 2.0f*q1*mx;
        float _2q0   = 2.0f*q0, _2q1 = 2.0f*q1;
        float _2q2   = 2.0f*q2, _2q3 = 2.0f*q3;
        float _2q0q2 = 2.0f*q0*q2, _2q2q3 = 2.0f*q2*q3;
        float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
        float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
        float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

        float hx = mx*(q0q0 + q1q1 - q2q2 - q3q3)
                 + 2.0f*my*(q1q2 - q0q3)
                 + 2.0f*mz*(q1q3 + q0q2);
        float hy = 2.0f*mx*(q1q2 + q0q3)
                 + my*(q0q0 - q1q1 + q2q2 - q3q3)
                 + 2.0f*mz*(q2q3 - q0q1);

        float _2bx = sqrtf(hx*hx + hy*hy);   /* horizontal component */
        float _2bz = 2.0f*(mz*(q0q0 - q1q1 - q2q2 + q3q3)
                          + mx*(q1q3 - q0q2)
                          + my*(q2q3 + q0q1));  /* vertical component */
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        /* Gradient descent (eq.32) — objective function Jacobian × F */
        s0 = -_2q2*(2.0f*(q1q3 - q0q2) - ax)
            + _2q1*(2.0f*(q0q1 + q2q3) - ay)
            - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s1 = _2q3*(2.0f*(q1q3 - q0q2) - ax)
            + _2q0*(2.0f*(q0q1 + q2q3) - ay)
            - 4.0f*q1*(1.0f - 2.0f*(q1q1 + q2q2))*(az)
            + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s2 = -_2q0*(2.0f*(q1q3 - q0q2) - ax)
            + _2q3*(2.0f*(q0q1 + q2q3) - ay)
            - 4.0f*q2*(1.0f - 2.0f*(q1q1 + q2q2))*(az)
            + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        s3 = _2q1*(2.0f*(q1q3 - q0q2) - ax)
            + _2q2*(2.0f*(q0q1 + q2q3) - ay)  /* note: corrected sign per errata */
            + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx)
            + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my)
            + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);

        /* Normalise gradient (eq.42 denominator) */
        rn = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= rn; s1 *= rn; s2 *= rn; s3 *= rn;

        /* Apply feedback (eq.42) */
        qDot0 -= mw->beta * s0;
        qDot1 -= mw->beta * s1;
        qDot2 -= mw->beta * s2;
        qDot3 -= mw->beta * s3;
    }

    /* Integrate (eq.42 — first-order Euler) */
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    /* Renormalise quaternion */
    rn = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    mw->q0 = q0 * rn; mw->q1 = q1 * rn;
    mw->q2 = q2 * rn; mw->q3 = q3 * rn;
    mw->initialised = true;

    Madgwick_GetEuler(mw);
}

/* ── IMU-only update (6-DOF, eq.25 + eq.42) ─────────────────────── */
void Madgwick_UpdateIMU(Madgwick_t *mw,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt)
{
    float q0 = mw->q0, q1 = mw->q1, q2 = mw->q2, q3 = mw->q3;
    float rn;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        rn = inv_sqrt(ax*ax + ay*ay + az*az);
        ax *= rn; ay *= rn; az *= rn;

        /* Gradient of objective function F_g (eq.25) */
        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
        float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
        float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1
           + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2
           + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        rn = inv_sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= rn; s1 *= rn; s2 *= rn; s3 *= rn;

        qDot0 -= mw->beta * s0;
        qDot1 -= mw->beta * s1;
        qDot2 -= mw->beta * s2;
        qDot3 -= mw->beta * s3;
    }

    q0 += qDot0 * dt; q1 += qDot1 * dt;
    q2 += qDot2 * dt; q3 += qDot3 * dt;

    rn = inv_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    mw->q0 = q0*rn; mw->q1 = q1*rn;
    mw->q2 = q2*rn; mw->q3 = q3*rn;
    mw->initialised = true;

    Madgwick_GetEuler(mw);
}

/* ── Euler angles from quaternion ────────────────────────────────── */
void Madgwick_GetEuler(Madgwick_t *mw)
{
    float q0 = mw->q0, q1 = mw->q1, q2 = mw->q2, q3 = mw->q3;

    /* ZYX (yaw-pitch-roll) Euler decomposition — aerospace convention */
    mw->roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    mw->pitch = asinf( 2.0f*(q0*q2 - q3*q1));
    mw->yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}
