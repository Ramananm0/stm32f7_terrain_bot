/**
 * motor.c — 4-motor velocity driver via PCA9685 (I2C PWM expander)
 * STM32F746G-DISCO
 *
 * WHY PCA9685 instead of TIM1 direct PWM?
 * ────────────────────────────────────────
 * TIM1_CH2 → PE11, TIM1_CH3 → PE13, TIM1_CH4 → PE14 are all used
 * internally by the LTDC peripheral for the built-in 4.3" LCD.
 * Only TIM1_CH1 (PA8 = Arduino D10) is free.
 * The PCA9685 routes all 4 motor PWM + 4 direction signals over
 * I2C1 (PB8/PB9), which is already wired to the ICM-20948.
 *
 * Channel assignment on PCA9685:
 *   CH0 : Motor FL PWM     CH4 : Motor FL DIR
 *   CH1 : Motor FR PWM     CH5 : Motor FR DIR
 *   CH2 : Motor RL PWM     CH6 : Motor RL DIR
 *   CH3 : Motor RR PWM     CH7 : Motor RR DIR
 *
 * DIR logic: PCA9685_SetDigital(ch, 0) = LOW = forward
 *            PCA9685_SetDigital(ch, 1) = HIGH = reverse
 */

#include "motor.h"
#include "pca9685.h"
#include <math.h>

#define PWM_CH_OFFSET   0u   /* CH0..CH3 = speed PWM  */
#define DIR_CH_OFFSET   4u   /* CH4..CH7 = direction  */

/*
 * Polarity correction for mirror-mounted right-side motors.
 * +1 = standard,  -1 = invert direction signal.
 */
static const int8_t MOTOR_POLARITY[MOTOR_NUM] = {
    +1,   /* FL — standard orientation */
    -1,   /* FR — mirror-mounted, invert */
    +1,   /* RL — standard orientation */
    -1,   /* RR — mirror-mounted, invert */
};

static uint8_t g_initialised = 0;

/* ── Public API ─────────────────────────────────────────────────── */

void Motor_Init(TIM_HandleTypeDef *htim)
{
    (void)htim;   /* TIM1 not used — PCA9685 handles all PWM channels */
    /* PCA9685_Init(&hi2c1) must be called in App_Run() before this */
    g_initialised = 1;
    Motor_StopAll();
}

void Motor_Set(Motor_ID id, float vel_mmps)
{
    if ((uint8_t)id >= MOTOR_NUM || !g_initialised) return;

    /* Apply polarity correction for mirrored motors */
    float v = vel_mmps * (float)MOTOR_POLARITY[id];

    /* Clamp */
    if (v >  MOTOR_MAX_SPEED_MMPS) v =  MOTOR_MAX_SPEED_MMPS;
    if (v < -MOTOR_MAX_SPEED_MMPS) v = -MOTOR_MAX_SPEED_MMPS;

    /* Direction pin: LOW = forward, HIGH = reverse */
    PCA9685_SetDigital((uint8_t)(DIR_CH_OFFSET + (uint8_t)id),
                       (v >= 0.0f) ? 0u : 1u);

    /* Speed PWM: 0–4095 proportional to |velocity| */
    float ratio = fabsf(v) / MOTOR_MAX_SPEED_MMPS;
    uint16_t duty = (uint16_t)(ratio * (float)PCA9685_MAX_DUTY);
    if (duty > PCA9685_MAX_DUTY) duty = PCA9685_MAX_DUTY;

    PCA9685_SetDuty((uint8_t)(PWM_CH_OFFSET + (uint8_t)id), duty);
}

void Motor_StopAll(void)
{
    if (!g_initialised) return;
    for (uint8_t i = 0u; i < (uint8_t)MOTOR_NUM; i++)
        PCA9685_SetDuty((uint8_t)(PWM_CH_OFFSET + i), 0u);
}
