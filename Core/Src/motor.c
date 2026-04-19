/**
 * motor.c — BTS7960 skid-steer motor driver
 * STM32F746G-DISCO
 *
 * LEFT  BTS7960: TIM12_CH1=RPWM(PH6/D6)   TIM12_CH2=LPWM(PB15/D11)
 * RIGHT BTS7960: TIM1_CH1 =RPWM(PA8/D10)  TIM13_CH1=LPWM(PA6/Morpho)
 */

#include "motor.h"
#include <math.h>

/* ── Timer handles ───────────────────────────────────────────────── */
static TIM_HandleTypeDef *g_tim12  = NULL;  /* LEFT  RPWM+LPWM */
static TIM_HandleTypeDef *g_tim1   = NULL;  /* RIGHT RPWM      */
static TIM_HandleTypeDef *g_tim13  = NULL;  /* RIGHT LPWM      */
static uint8_t g_ready = 0;

/* ── Internal: set one PWM channel ──────────────────────────────── */
static inline void set_pwm(TIM_HandleTypeDef *htim,
                            uint32_t ch, uint32_t duty)
{
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

/* ── Public API ──────────────────────────────────────────────────── */

void Motor_Init(TIM_HandleTypeDef *htim12,
                TIM_HandleTypeDef *htim1,
                TIM_HandleTypeDef *htim13)
{
    g_tim12 = htim12;
    g_tim1  = htim1;
    g_tim13 = htim13;

    /* Start all 4 PWM outputs */
    HAL_TIM_PWM_Start(g_tim12, TIM_CHANNEL_1);   /* LEFT  RPWM */
    HAL_TIM_PWM_Start(g_tim12, TIM_CHANNEL_2);   /* LEFT  LPWM */
    HAL_TIM_PWM_Start(g_tim1,  TIM_CHANNEL_1);   /* RIGHT RPWM */
    HAL_TIM_PWM_Start(g_tim13, TIM_CHANNEL_1);   /* RIGHT LPWM */

    g_ready = 1;
    Motor_StopAll();
}

void Motor_SetSide(Motor_Side side, float vel_mmps)
{
    if (!g_ready) return;

    /* Clamp to physical limit */
    if (vel_mmps >  MOTOR_MAX_SPEED_MMPS) vel_mmps =  MOTOR_MAX_SPEED_MMPS;
    if (vel_mmps < -MOTOR_MAX_SPEED_MMPS) vel_mmps = -MOTOR_MAX_SPEED_MMPS;

    /* Convert speed → PWM duty (0 to ARR) */
    uint32_t duty = (uint32_t)(fabsf(vel_mmps) / MOTOR_MAX_SPEED_MMPS
                               * (float)MOTOR_PWM_ARR);
    if (duty > MOTOR_PWM_ARR) duty = MOTOR_PWM_ARR;

    if (side == SIDE_LEFT) {
        if (vel_mmps >= 0.0f) {
            set_pwm(g_tim12, TIM_CHANNEL_2, 0);      /* LPWM = 0   */
            set_pwm(g_tim12, TIM_CHANNEL_1, duty);   /* RPWM = duty */
        } else {
            set_pwm(g_tim12, TIM_CHANNEL_1, 0);      /* RPWM = 0   */
            set_pwm(g_tim12, TIM_CHANNEL_2, duty);   /* LPWM = duty */
        }
    } else {  /* SIDE_RIGHT */
        if (vel_mmps >= 0.0f) {
            set_pwm(g_tim13, TIM_CHANNEL_1, 0);      /* LPWM = 0   */
            set_pwm(g_tim1,  TIM_CHANNEL_1, duty);   /* RPWM = duty */
        } else {
            set_pwm(g_tim1,  TIM_CHANNEL_1, 0);      /* RPWM = 0   */
            set_pwm(g_tim13, TIM_CHANNEL_1, duty);   /* LPWM = duty */
        }
    }
}

void Motor_StopAll(void)
{
    if (!g_ready) return;
    set_pwm(g_tim12, TIM_CHANNEL_1, 0);
    set_pwm(g_tim12, TIM_CHANNEL_2, 0);
    set_pwm(g_tim1,  TIM_CHANNEL_1, 0);
    set_pwm(g_tim13, TIM_CHANNEL_1, 0);
}
