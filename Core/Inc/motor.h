/**
 * motor.h — 4-motor PWM velocity driver (via PCA9685)
 * Board : STM32F746G-DISCO
 *
 * All motor PWM and direction signals are routed through a PCA9685
 * 16-channel I2C PWM controller on I2C1 (PB8=SCL, PB9=SDA).
 *
 * This avoids conflicts with the built-in LCD (LTDC uses PE11/PE13/PE14
 * which overlap with TIM1_CH2/CH3/CH4) and with FMC SDRAM (PI2/PI3).
 *
 * ── PCA9685 channel assignment ───────────────────────────────────────
 *   CH0 : Motor FL  PWM   (speed,  0–4095)
 *   CH1 : Motor FR  PWM
 *   CH2 : Motor RL  PWM
 *   CH3 : Motor RR  PWM
 *   CH4 : Motor FL  DIR   (0=fwd LOW,  4095=rev HIGH)
 *   CH5 : Motor FR  DIR
 *   CH6 : Motor RL  DIR
 *   CH7 : Motor RR  DIR
 *
 * ── H-bridge wiring per motor (e.g. L298N, TB6612FNG, DRV8833) ──────
 *   PCA9685 CH_PWM  →  ENA  (enable / speed)
 *   PCA9685 CH_DIR  →  IN1  (direction input 1)
 *   GND             →  IN2  (keep LOW for single-pin direction logic)
 *   Motor terminals →  Motor A / B
 *
 * ── Polarity ─────────────────────────────────────────────────────────
 *   Right-side motors (FR, RR) are physically mirror-mounted.
 *   Firmware handles reversal via MOTOR_POLARITY[] in motor.c.
 *   If a wheel spins backwards, change its entry from -1 to +1.
 *
 * ── CubeMX: No TIM1 configuration needed for motor PWM ──────────────
 *   I2C1 must be configured at 400 kHz (already used for ICM-20948).
 *   PCA9685_Init() must be called before Motor_Init().
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Motor indices ────────────────────────────────────────────────── */
typedef enum {
    MOTOR_FL  = 0,   /* Front-Left  */
    MOTOR_FR  = 1,   /* Front-Right */
    MOTOR_RL  = 2,   /* Rear-Left   */
    MOTOR_RR  = 3,   /* Rear-Right  */
    MOTOR_NUM = 4
} Motor_ID;

/* ── Physical speed limit ─────────────────────────────────────────── */
/* RMCS-3070 @ 12 V: 100 RPM × π × 8.5 mm wheel / 60 s = 44.5 mm/s  */
#define MOTOR_MAX_SPEED_MMPS   44.5f

/* ── Anti-windup clamp for PI integrator ─────────────────────────── */
#define MOTOR_ILIMIT   (MOTOR_MAX_SPEED_MMPS * 0.5f)   /* 22.25 mm/s */

/* ── API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise motor driver. Call after PCA9685_Init().
 *         The htim parameter is unused (kept for API compatibility).
 */
void Motor_Init(TIM_HandleTypeDef *htim);

/**
 * @brief  Drive one motor at the requested velocity.
 * @param  id        Motor index (MOTOR_FL … MOTOR_RR)
 * @param  vel_mmps  +ve = forward, -ve = reverse. Clamped to ±44.5 mm/s.
 */
void Motor_Set(Motor_ID id, float vel_mmps);

/**
 * @brief  Immediately stop all four motors (zero PWM duty).
 *         Direction pins are not changed.
 */
void Motor_StopAll(void);

#endif /* MOTOR_H */
