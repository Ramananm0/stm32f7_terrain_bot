/**
 * motor.h — 2× BTS7960 skid-steer motor driver
 * STM32F746G-DISCO (Arduino header only)
 *
 * ── BTS7960 wiring ───────────────────────────────────────────────────
 *
 *   BTS7960 #1 = LEFT  side (FL + RL motors in parallel)
 *   BTS7960 #2 = RIGHT side (FR + RR motors in parallel)
 *
 *   Each BTS7960:
 *     RPWM → forward PWM  (STM32 timer)
 *     LPWM → reverse PWM  (STM32 timer)
 *     R_EN → tie to 3.3V  (always enabled)
 *     L_EN → tie to 3.3V  (always enabled)
 *     RIS  → leave unconnected (current sense, optional)
 *     LIS  → leave unconnected
 *     VCC  → 5V
 *     GND  → GND (common with STM32)
 *
 * ── PWM Pin Assignment (all Arduino header) ──────────────────────────
 *
 *   LEFT  BTS7960:
 *     RPWM ← A4 (PF7)  TIM11_CH1  forward
 *     LPWM ← A5 (PF6)  TIM10_CH1  reverse
 *
 *   RIGHT BTS7960:
 *     RPWM ← A2 (PF9)  TIM14_CH1  forward
 *     LPWM ← A3 (PF8)  TIM13_CH1  reverse
 *
 * ── CubeMX settings ──────────────────────────────────────────────────
 *   TIM10: PWM CH1, pin=PF6, PSC=0, ARR=10799 (20kHz), Mode1, High
 *   TIM11: PWM CH1, pin=PF7, PSC=0, ARR=10799 (20kHz), Mode1, High
 *   TIM13: PWM CH1, pin=PF8, PSC=0, ARR=10799 (20kHz), Mode1, High
 *   TIM14: PWM CH1, pin=PF9, PSC=0, ARR=10799 (20kHz), Mode1, High
 *
 * ── BTS7960 direction logic ──────────────────────────────────────────
 *   Forward : RPWM = duty (0-ARR),  LPWM = 0
 *   Reverse : RPWM = 0,             LPWM = duty
 *   Stop    : RPWM = 0,             LPWM = 0
 *
 * ── 2WD → 4WD upgrade ────────────────────────────────────────────────
 *   2WD: connect 1 motor per BTS (FL on BTS#1, FR on BTS#2)
 *   4WD: add RL to BTS#1 output, RR to BTS#2 output
 *   NO firmware change needed — just wiring change!
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Physical speed limit ────────────────────────────────────────── */
/* RMCS-3070 @ 12V: 100 RPM × π × 8.5mm / 60s = 44.5 mm/s          */
#define MOTOR_MAX_SPEED_MMPS   44.5f

/* ARR value — must match CubeMX TIM10/11/13/14 ARR setting          */
/* PSC=0, ARR=10799 → 216MHz / 10800 = 20kHz PWM                    */
#define MOTOR_PWM_ARR          10799u

/* Anti-windup clamp for velocity PI integrator                       */
#define MOTOR_ILIMIT           (MOTOR_MAX_SPEED_MMPS * 0.5f)

/* ── Side identifiers ────────────────────────────────────────────── */
typedef enum {
    SIDE_LEFT  = 0,   /* BTS7960 #1 → FL + RL motors */
    SIDE_RIGHT = 1,   /* BTS7960 #2 → FR + RR motors */
    SIDE_NUM   = 2
} Motor_Side;

/* ── API ─────────────────────────────────────────────────────────── */

/**
 * @brief Init both BTS7960 drivers, start PWM outputs.
 *        Call after MX_TIM10/11/13/14_Init().
 *
 * @param htim10  TIM10 handle (LEFT  LPWM = A5/PF6)
 * @param htim11  TIM11 handle (LEFT  RPWM = A4/PF7)
 * @param htim13  TIM13 handle (RIGHT LPWM = A3/PF8)
 * @param htim14  TIM14 handle (RIGHT RPWM = A2/PF9)
 */
void Motor_Init(TIM_HandleTypeDef *htim10,
                TIM_HandleTypeDef *htim11,
                TIM_HandleTypeDef *htim13,
                TIM_HandleTypeDef *htim14);

/**
 * @brief Set velocity for one side.
 * @param side      SIDE_LEFT or SIDE_RIGHT
 * @param vel_mmps  +ve=forward, -ve=reverse. Clamped ±44.5 mm/s.
 */
void Motor_SetSide(Motor_Side side, float vel_mmps);

/**
 * @brief Stop both sides immediately (zero duty on all PWM pins).
 */
void Motor_StopAll(void);

/**
 * @brief Get current direction of a side.
 * @return 1=forward, -1=reverse, 0=stopped
 */
int8_t Motor_GetDirection(Motor_Side side);

#endif /* MOTOR_H */
