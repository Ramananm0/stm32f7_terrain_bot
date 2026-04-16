/**
 * motor.h — 4-motor PWM velocity driver
 * Board : STM32F746G-DISCO
 *
 * ── CubeMX: TIM1 (Motor PWM, 20 kHz) ────────────────────────────────
 *   Mode               : PWM Generation CH1 / CH2 / CH3 / CH4
 *   Internal Clock
 *   Prescaler (PSC)    : 0            (TIM1 clock = 216 MHz on APB2)
 *   Counter Period(ARR): 10799        → 216 MHz / 10800 = 20 kHz
 *   Auto-reload preload: Enable
 *   CH1..4 Mode        : PWM mode 1,  Polarity: High
 *
 *   Pin mapping (CubeMX):
 *     TIM1_CH1 → PA8  (Arduino D10)       Motor FL  (Front-Left)
 *     TIM1_CH2 → PE11 (Morpho CN1 pin 6)  Motor FR  (Front-Right)
 *     TIM1_CH3 → PE13 (Morpho CN1 pin 8)  Motor RL  (Rear-Left)
 *     TIM1_CH4 → PE14 (Morpho CN1 pin 10) Motor RR  (Rear-Right)
 *
 * ── CubeMX: Direction GPIO ──────────────────────────────────────────
 *   Configure each as: GPIO_Output, Push-Pull, No Pull, Low speed.
 *   Initial output level: Low.
 *
 *   Default assignment (change MOTOR_xx_DIR_PORT/PIN macros below):
 *     FL_DIR → PG6  (Arduino D2)
 *     FR_DIR → PG7  (Arduino D4)
 *     RL_DIR → PI3  (Arduino D7)
 *     RR_DIR → PI2  (Arduino D8)
 *
 * ── H-bridge wiring per motor ────────────────────────────────────────
 *   One driver chip per motor (e.g. L298N, TB6612FNG, DRV8833):
 *     PWM pin → ENA  (enable / speed)
 *     DIR pin → IN1  (direction input 1)
 *     GND     → IN2  (keep LOW for single-direction H-bridge logic)
 *
 * ── Motor polarity ───────────────────────────────────────────────────
 *   Right-side motors are physically mirrored.
 *   Adjust MOTOR_FL/FR/RL/RR_POLARITY in motor.c if a wheel spins
 *   backwards when it should go forward.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Motor indices ──────────────────────────────────────────────────── */
typedef enum {
    MOTOR_FL  = 0,   /* Front-Left  */
    MOTOR_FR  = 1,   /* Front-Right */
    MOTOR_RL  = 2,   /* Rear-Left   */
    MOTOR_RR  = 3,   /* Rear-Right  */
    MOTOR_NUM = 4
} Motor_ID;

/* ── Physical speed limit ───────────────────────────────────────────── */
/* RMCS-3070 @ 12 V: 100 RPM output × 26.70 mm wheel circ / 60 s        */
/* Update if you mount larger wheels.                                     */
#define MOTOR_MAX_SPEED_MMPS   44.5f

/* ── PWM timer constant ─────────────────────────────────────────────── */
/* Must equal the ARR value set in CubeMX for TIM1.                      */
#define MOTOR_PWM_ARR          10799u

/* ── Direction GPIO ──────────────────────────────────────────────────── */
/* Edit port/pin to match your CubeMX label assignments.                  */
#define MOTOR_FL_DIR_PORT   GPIOG
#define MOTOR_FL_DIR_PIN    GPIO_PIN_6    /* D2  PG6 */

#define MOTOR_FR_DIR_PORT   GPIOG
#define MOTOR_FR_DIR_PIN    GPIO_PIN_7    /* D4  PG7 */

#define MOTOR_RL_DIR_PORT   GPIOI
#define MOTOR_RL_DIR_PIN    GPIO_PIN_3    /* D7  PI3 */

#define MOTOR_RR_DIR_PORT   GPIOI
#define MOTOR_RR_DIR_PIN    GPIO_PIN_2    /* D8  PI2 */

/* ── TIM1 channel per motor ─────────────────────────────────────────── */
#define MOTOR_FL_TIM_CH    TIM_CHANNEL_1
#define MOTOR_FR_TIM_CH    TIM_CHANNEL_2
#define MOTOR_RL_TIM_CH    TIM_CHANNEL_3
#define MOTOR_RR_TIM_CH    TIM_CHANNEL_4

/* ── API ────────────────────────────────────────────────────────────── */

/**
 * @brief Initialise motor PWM outputs. Call after MX_TIM1_Init().
 * @param htim1  TIM1 handle (4-channel PWM, configured in CubeMX).
 */
void Motor_Init(TIM_HandleTypeDef *htim1);

/**
 * @brief Drive one motor at the requested velocity.
 *        Velocity is linearly mapped to PWM duty cycle.
 *        Input is clamped to ±MOTOR_MAX_SPEED_MMPS.
 *
 * @param id        Motor index (MOTOR_FL … MOTOR_RR).
 * @param vel_mmps  Velocity mm/s. +ve = forward, -ve = reverse.
 */
void Motor_Set(Motor_ID id, float vel_mmps);

/**
 * @brief Immediately zero PWM on all four motors (coast to stop).
 *        Direction pins are not changed.
 */
void Motor_StopAll(void);

#endif /* MOTOR_H */
