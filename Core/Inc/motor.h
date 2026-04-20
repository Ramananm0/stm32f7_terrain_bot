/**
 * motor.h — 4× BTS7960 individual motor driver (true 4WD)
 * STM32F746G-DISCO
 *
 * ── Voltage protection ───────────────────────────────────────────────
 *   Battery: 12.4V nominal, up to 14V fully charged
 *   Motor rated: 12V (RMCS-3070)
 *   Max safe duty: 12V / 14V × ARR = 85.7% × 10799 = 9255
 *   This limits effective voltage to 12V at full speed.
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
 * ── Pin assignment (all Arduino header) ──────────────────────────────
 *   FL: RPWM=A4(PF7/TIM11_CH1)  LPWM=A5(PF6/TIM10_CH1)
 *   FR: RPWM=A2(PF9/TIM14_CH1)  LPWM=A3(PF8/TIM13_CH1)
 *   RL: RPWM=D6(PH6/TIM12_CH1)  LPWM=D11(PB15/TIM12_CH2)
 *   RR: RPWM=D10(PA8/TIM1_CH1)  LPWM=D3(PB4/TIM3_CH1)
 *
 * ── CubeMX settings ──────────────────────────────────────────────────
 *   TIM10/11/13/14: PWM CH1, PSC=0, ARR=10799, Mode1, High
 *   TIM12: PWM CH1+CH2, PSC=0, ARR=10799, Mode1, High
 *   TIM1:  PWM CH1 only, PSC=0, ARR=10799, Mode1, High
 *   TIM3:  PWM CH1 only, PSC=0, ARR=10799, Mode1, High
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
/* PSC=0, ARR=10799 → 216MHz/10800 = 20kHz                           */
#define MOTOR_PWM_ARR          10799u

/* ── Voltage protection ──────────────────────────────────────────── */
/* Battery max 14V, motor rated 12V                                   */
/* Max duty = 12/14 × 10799 = 9256                                   */
#define MOTOR_BATTERY_MAX_V    14.0f
#define MOTOR_RATED_V          12.0f
#define MOTOR_MAX_DUTY         ((uint32_t)(MOTOR_PWM_ARR * \
                                 (MOTOR_RATED_V / MOTOR_BATTERY_MAX_V)))

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
                TIM_HandleTypeDef *htim1,
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
