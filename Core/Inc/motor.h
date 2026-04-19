/**
 * motor.h — BTS7960 dual H-bridge driver (skid-steer / tank drive)
 * STM32F746G-DISCO
 *
 * ── Drive topology ───────────────────────────────────────────────────
 *   BTS7960 #1 = LEFT  side → FL + RL motors wired in parallel
 *   BTS7960 #2 = RIGHT side → FR + RR motors wired in parallel
 *
 *   All 4 encoders are read individually for accurate odometry.
 *   Velocity commands are per-side (left / right).
 *
 * ── BTS7960 pin explanation ──────────────────────────────────────────
 *   Each BTS7960 module has 2 PWM inputs:
 *     RPWM → controls forward  half-bridge (motor spins forward)
 *     LPWM → controls reverse  half-bridge (motor spins reverse)
 *
 *   Forward : RPWM = duty (0–ARR),  LPWM = 0
 *   Reverse : RPWM = 0,             LPWM = duty
 *   Stop    : RPWM = 0,             LPWM = 0
 *
 *   R_EN and L_EN → tie both to 3.3V (always enabled)
 *   VCC → 5V,  GND → common GND,  B+/B- → motor,  M+/M- → 12V
 *
 * ── STM32 PWM pin assignment ─────────────────────────────────────────
 *   Using TIM12 for LEFT and TIM1_CH1 + TIM13 for RIGHT.
 *   These pins are FREE (no LTDC/SDRAM conflict).
 *
 *   LEFT  BTS7960 #1:
 *     RPWM ← TIM12_CH1 → PH6  (Arduino D6)
 *     LPWM ← TIM12_CH2 → PB15 (Arduino D11)
 *
 *   RIGHT BTS7960 #2:
 *     RPWM ← TIM1_CH1  → PA8  (Arduino D10)
 *     LPWM ← TIM13_CH1 → PA6  (Morpho CN1)
 *
 * ── CubeMX settings ──────────────────────────────────────────────────
 *   TIM12: PWM Generation CH1+CH2
 *          CH1=PH6, CH2=PB15
 *          PSC=0, ARR=10799, Mode1, Polarity High
 *
 *   TIM1:  PWM Generation CH1
 *          CH1=PA8
 *          PSC=0, ARR=10799, Mode1, Polarity High
 *
 *   TIM13: PWM Generation CH1
 *          CH1=PA6
 *          PSC=0, ARR=10799, Mode1, Polarity High
 *
 *   All → Auto-reload preload: Enable
 *   PSC=0, ARR=10799 → 216 MHz / 10800 = 20 kHz PWM
 *
 * ── 2WD → 4WD upgrade ────────────────────────────────────────────────
 *   Right now (2WD): Connect only 1 motor per BTS7960 (e.g. FL + FR)
 *   After pin extender arrives (4WD): Add RL to BTS#1, RR to BTS#2
 *   NO firmware change needed — wiring change only.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Physical constants ──────────────────────────────────────────── */
/* RMCS-3070 @ 12V: 100 RPM × π × 8.5 mm wheel / 60 s = 44.5 mm/s  */
#define MOTOR_MAX_SPEED_MMPS   44.5f

/* ARR value in CubeMX for all motor PWM timers (20 kHz @ 216 MHz)   */
#define MOTOR_PWM_ARR          10799u

/* Anti-windup clamp for velocity PI integrator                       */
#define MOTOR_ILIMIT           (MOTOR_MAX_SPEED_MMPS * 0.5f)

/* ── Side identifiers ────────────────────────────────────────────── */
typedef enum {
    SIDE_LEFT  = 0,   /* BTS7960 #1 → FL + RL */
    SIDE_RIGHT = 1,   /* BTS7960 #2 → FR + RR */
    SIDE_NUM   = 2
} Motor_Side;

/* ── API ─────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise both BTS7960 drivers and start PWM outputs.
 *         Call after MX_TIM12_Init(), MX_TIM1_Init(), MX_TIM13_Init().
 * @param  htim12  TIM12 handle (LEFT  RPWM=CH1, LPWM=CH2)
 * @param  htim1   TIM1  handle (RIGHT RPWM=CH1)
 * @param  htim13  TIM13 handle (RIGHT LPWM=CH1)
 */
void Motor_Init(TIM_HandleTypeDef *htim12,
                TIM_HandleTypeDef *htim1,
                TIM_HandleTypeDef *htim13);

/**
 * @brief  Set velocity for one side (left or right).
 *         Both motors on that side run at the same speed.
 * @param  side      SIDE_LEFT or SIDE_RIGHT
 * @param  vel_mmps  +ve = forward, -ve = reverse. Clamped ±44.5 mm/s.
 */
void Motor_SetSide(Motor_Side side, float vel_mmps);

/**
 * @brief  Immediately stop both sides (zero duty on all 4 PWM pins).
 */
void Motor_StopAll(void);

#endif /* MOTOR_H */
