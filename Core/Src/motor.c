/**
 * motor.c — 2× BTS7960 skid-steer driver
 * STM32F746G-DISCO
 *
 * LEFT  BTS7960: TIM11_CH1(A4/PF7)=RPWM  TIM10_CH1(A5/PF6)=LPWM
 * RIGHT BTS7960: TIM14_CH1(A2/PF9)=RPWM  TIM13_CH1(A3/PF8)=LPWM
 *
 * BTS7960 logic:
 *   Forward : RPWM = duty,  LPWM = 0
 *   Reverse : RPWM = 0,     LPWM = duty
 *   Stop    : RPWM = 0,     LPWM = 0
 */

#include "motor.h"
#include "encoder.h"
#include <math.h>

/* Timer handles */
static TIM_HandleTypeDef *g_tim10 = NULL;  /* LEFT  LPWM (A5/PF6) */
static TIM_HandleTypeDef *g_tim11 = NULL;  /* LEFT  RPWM (A4/PF7) */
static TIM_HandleTypeDef *g_tim13 = NULL;  /* RIGHT LPWM (A3/PF8) */
static TIM_HandleTypeDef *g_tim14 = NULL;  /* RIGHT RPWM (A2/PF9) */

static uint8_t  g_ready = 0;
static int8_t   g_dir[SIDE_NUM] = {0, 0};  /* current direction per side */

/* ── Internal PWM setter ─────────────────────────────────────────── */
static inline void set_pwm(TIM_HandleTypeDef *htim,
                            uint32_t ch, uint32_t duty)
{
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

/* ── Public API ──────────────────────────────────────────────────── */

void Motor_Init(TIM_HandleTypeDef *htim10,
                TIM_HandleTypeDef *htim11,
                TIM_HandleTypeDef *htim13,
                TIM_HandleTypeDef *htim14)
{
    g_tim10 = htim10;
    g_tim11 = htim11;
    g_tim13 = htim13;
    g_tim14 = htim14;

    /* Start all 4 PWM channels */
    HAL_TIM_PWM_Start(g_tim10, TIM_CHANNEL_1);  /* LEFT  LPWM */
    HAL_TIM_PWM_Start(g_tim11, TIM_CHANNEL_1);  /* LEFT  RPWM */
    HAL_TIM_PWM_Start(g_tim13, TIM_CHANNEL_1);  /* RIGHT LPWM */
    HAL_TIM_PWM_Start(g_tim14, TIM_CHANNEL_1);  /* RIGHT RPWM */

    g_ready = 1;
    Motor_StopAll();
}

void Motor_SetSide(Motor_Side side, float vel_mmps)
{
    if (!g_ready) return;

    /* Clamp to physical limit */
    if (vel_mmps >  MOTOR_MAX_SPEED_MMPS) vel_mmps =  MOTOR_MAX_SPEED_MMPS;
    if (vel_mmps < -MOTOR_MAX_SPEED_MMPS) vel_mmps = -MOTOR_MAX_SPEED_MMPS;

    /* Convert speed to PWM duty (0 to ARR) */
    uint32_t duty = (uint32_t)(fabsf(vel_mmps) / MOTOR_MAX_SPEED_MMPS
                               * (float)MOTOR_PWM_ARR);
    if (duty > MOTOR_PWM_ARR) duty = MOTOR_PWM_ARR;

    /* Update encoder direction */
    uint8_t fwd = (vel_mmps >= 0.0f) ? 1u : 0u;

    if (side == SIDE_LEFT) {
        g_dir[SIDE_LEFT] = (vel_mmps > 0.0f) ? 1 :
                           (vel_mmps < 0.0f) ? -1 : 0;
        /* Update FL and RL encoder directions */
        Encoder_SetDirection(ENC_FL, fwd);
        Encoder_SetDirection(ENC_RL, fwd);

        if (vel_mmps >= 0.0f) {
            /* Forward: RPWM=duty, LPWM=0 */
            set_pwm(g_tim10, TIM_CHANNEL_1, 0);      /* LPWM = 0    */
            set_pwm(g_tim11, TIM_CHANNEL_1, duty);   /* RPWM = duty */
        } else {
            /* Reverse: RPWM=0, LPWM=duty */
            set_pwm(g_tim11, TIM_CHANNEL_1, 0);      /* RPWM = 0    */
            set_pwm(g_tim10, TIM_CHANNEL_1, duty);   /* LPWM = duty */
        }
    } else {  /* SIDE_RIGHT */
        g_dir[SIDE_RIGHT] = (vel_mmps > 0.0f) ? 1 :
                            (vel_mmps < 0.0f) ? -1 : 0;
        /* Update FR and RR encoder directions */
        Encoder_SetDirection(ENC_FR, fwd);
        Encoder_SetDirection(ENC_RR, fwd);

        if (vel_mmps >= 0.0f) {
            /* Forward: RPWM=duty, LPWM=0 */
            set_pwm(g_tim13, TIM_CHANNEL_1, 0);      /* LPWM = 0    */
            set_pwm(g_tim14, TIM_CHANNEL_1, duty);   /* RPWM = duty */
        } else {
            /* Reverse: RPWM=0, LPWM=duty */
            set_pwm(g_tim14, TIM_CHANNEL_1, 0);      /* RPWM = 0    */
            set_pwm(g_tim13, TIM_CHANNEL_1, duty);   /* LPWM = duty */
        }
    }
}

void Motor_StopAll(void)
{
    if (!g_ready) return;
    set_pwm(g_tim10, TIM_CHANNEL_1, 0);
    set_pwm(g_tim11, TIM_CHANNEL_1, 0);
    set_pwm(g_tim13, TIM_CHANNEL_1, 0);
    set_pwm(g_tim14, TIM_CHANNEL_1, 0);
    g_dir[SIDE_LEFT]  = 0;
    g_dir[SIDE_RIGHT] = 0;
    Encoder_SetDirection(ENC_FL, 1);
    Encoder_SetDirection(ENC_FR, 1);
    Encoder_SetDirection(ENC_RL, 1);
    Encoder_SetDirection(ENC_RR, 1);
}

int8_t Motor_GetDirection(Motor_Side side)
{
    if ((uint8_t)side >= SIDE_NUM) return 0;
    return g_dir[side];
}
