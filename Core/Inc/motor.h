/**
 * motor.h — 4× BTS7960 individual motor driver (true 4WD)
 * STM32F746G-DISCO
 *
 * ── Voltage protection ───────────────────────────────────────────────
 *   Battery: 12.4V nominal, up to 14V fully charged
 *   Motor rated: 12V (RMCS-3070)
 *   Duty cap = 12V / 14V = 85.7%
 *
 *   APB2 timers:
 *     TIM10 / TIM11  ARR = 10799  max CCR = 9254
 *
 *   APB1 timers:
 *     TIM2 / TIM3 / TIM12 / TIM13 / TIM14  ARR = 5399  max CCR = 4626
 *
 *   This limits effective motor voltage to about 12V at full battery charge.
 *
 * ── Soft start ───────────────────────────────────────────────────────
 *   PWM ramps from current duty to target over RAMP_MS milliseconds.
 *   Prevents current spikes on motor startup.
 *
 * ── BTS7960 wiring per motor ─────────────────────────────────────────
 *   RPWM → forward PWM  (STM32 timer)
 *   LPWM → reverse PWM  (STM32 timer)
 *   R_EN → 3.3V (always HIGH)
 *   L_EN → 3.3V (always HIGH)
 *   RIS  → not connected
 *   LIS  → not connected
 *
 * ── Pin assignment ───────────────────────────────────────────────────
 *   FL: RPWM=PF7 (TIM11_CH1)  LPWM=PF6  (TIM10_CH1)
 *   FR: RPWM=PF9 (TIM14_CH1)  LPWM=PF8  (TIM13_CH1)
 *   RL: RPWM=PH6 (TIM12_CH1)  LPWM=PB15 (TIM12_CH2)
 *   RR: RPWM=PA15(TIM2_CH1)   LPWM=PB4  (TIM3_CH1)
 *
 * ── CubeMX settings ──────────────────────────────────────────────────
 *   TIM10: PWM CH1, PSC=0, ARR=10799, Mode1, High
 *   TIM11: PWM CH1, PSC=0, ARR=10799, Mode1, High
 *   TIM2 : PWM CH1, PSC=0, ARR=5399,  Mode1, High
 *   TIM3 : PWM CH1, PSC=0, ARR=5399,  Mode1, High
 *   TIM12: PWM CH1+CH2, PSC=0, ARR=5399, Mode1, High
 *   TIM13: PWM CH1, PSC=0, ARR=5399,  Mode1, High
 *   TIM14: PWM CH1, PSC=0, ARR=5399,  Mode1, High
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Motor indices ───────────────────────────────────────────────── */
typedef enum {
    MOTOR_FL  = 0,
    MOTOR_FR  = 1,
    MOTOR_RL  = 2,
    MOTOR_RR  = 3,
    MOTOR_NUM = 4
} Motor_ID;

/* ── Physical speed limit ────────────────────────────────────────── */
#define MOTOR_MAX_SPEED_MMPS   44.5f

/* ── PWM constants ───────────────────────────────────────────────── */
#define MOTOR_PWM_ARR_APB2     10799u
#define MOTOR_PWM_ARR_APB1      5399u

/* ── Voltage protection ──────────────────────────────────────────── */
#define MOTOR_BATTERY_MAX_V    14.0f
#define MOTOR_RATED_V          12.0f

/* Safe integer caps matching 85.7% */
#define MOTOR_MAX_DUTY_APB2    9254u
#define MOTOR_MAX_DUTY_APB1    4626u

/* ── Soft start ──────────────────────────────────────────────────── */
/* Ramp time in ms — duty changes by max RAMP_STEP per PID cycle     */
#define MOTOR_RAMP_STEP        200u    /* max duty change per 10ms    */

/* ── Anti-windup ─────────────────────────────────────────────────── */
#define MOTOR_ILIMIT           (MOTOR_MAX_SPEED_MMPS * 0.5f)

/* ── API ─────────────────────────────────────────────────────────── */

/**
 * @brief Init all 4 BTS7960 motor drivers.
 *        Call after all MX_TIMxx_Init() calls.
 */
void Motor_Init(TIM_HandleTypeDef *htim10,
                TIM_HandleTypeDef *htim11,
                TIM_HandleTypeDef *htim12,
                TIM_HandleTypeDef *htim13,
                TIM_HandleTypeDef *htim14,
                TIM_HandleTypeDef *htim2,
                TIM_HandleTypeDef *htim3);

/**
 * @brief Set velocity for one motor individually.
 * @param id        MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR
 * @param vel_mmps  +ve=forward, -ve=reverse. Clamped ±44.5 mm/s.
 */
void Motor_Set(Motor_ID id, float vel_mmps);

/**
 * @brief Stop all motors immediately.
 */
void Motor_StopAll(void);

/**
 * @brief Get current direction of a motor.
 * @return 1=forward, -1=reverse, 0=stopped
 */
int8_t Motor_GetDirection(Motor_ID id);

#endif /* MOTOR_H */
