/**
 * motor.c — 4× BTS7960 individual motor driver (true 4WD)
 * STM32F746G-DISCO
 *
 * FL: TIM11(RPWM/A4) + TIM10(LPWM/A5)
 * FR: TIM14(RPWM/A2) + TIM13(LPWM/A3)
 * RL: TIM12_CH1(RPWM/D6) + TIM12_CH2(LPWM/D11)
 * RR: TIM1_CH1(RPWM/D10) + TIM3_CH1(LPWM/D3)
 *
 * Voltage protection: max duty = 9256/10799 = 85.7%
 * (12V rated motor, 14V max battery)
 * Soft start: duty ramps by max 200 counts per 10ms cycle
 */

#include "motor.h"
#include "encoder.h"
#include <math.h>
#include <string.h>

/* ── Timer handles ───────────────────────────────────────────────── */
static TIM_HandleTypeDef *g_tim10  = NULL; /* FL LPWM  A5  PF6 */
static TIM_HandleTypeDef *g_tim11  = NULL; /* FL RPWM  A4  PF7 */
static TIM_HandleTypeDef *g_tim12  = NULL; /* RL RPWM+LPWM D6+D11 */
static TIM_HandleTypeDef *g_tim13  = NULL; /* FR LPWM  A3  PF8 */
static TIM_HandleTypeDef *g_tim14  = NULL; /* FR RPWM  A2  PF9 */
static TIM_HandleTypeDef *g_tim1   = NULL; /* RR RPWM  D10 PA8 */
static TIM_HandleTypeDef *g_tim3   = NULL; /* RR LPWM  D3  PB4 */

static uint8_t  g_ready = 0;
static int8_t   g_dir[MOTOR_NUM]          = {0};
static uint32_t g_current_duty[MOTOR_NUM] = {0};

/* ── Set PWM helper ──────────────────────────────────────────────── */
static inline void set_pwm(TIM_HandleTypeDef *htim,
                            uint32_t ch, uint32_t duty)
{
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

/* ── Apply duty with soft start ramp ────────────────────────────── */
static uint32_t ramp_duty(uint32_t current, uint32_t target)
{
    if (target > current) {
        uint32_t step = target - current;
        if (step > MOTOR_RAMP_STEP) step = MOTOR_RAMP_STEP;
        return current + step;
    } else if (target < current) {
        uint32_t step = current - target;
        if (step > MOTOR_RAMP_STEP) step = MOTOR_RAMP_STEP;
        return current - step;
    }
    return current;
}

/* ── Set one motor ───────────────────────────────────────────────── */
static void set_motor_raw(Motor_ID id, uint32_t duty, uint8_t forward)
{
    /* Clamp to voltage-protected max */
    if (duty > MOTOR_MAX_DUTY) duty = MOTOR_MAX_DUTY;

    /* Apply soft start ramp */
    duty = ramp_duty(g_current_duty[id], duty);
    g_current_duty[id] = duty;

    /* Update encoder direction */
    Encoder_SetDirection((uint8_t)id, forward);

    switch (id) {
        case MOTOR_FL:
            if (forward) {
                set_pwm(g_tim10, TIM_CHANNEL_1, 0);
                set_pwm(g_tim11, TIM_CHANNEL_1, duty);
            } else {
                set_pwm(g_tim11, TIM_CHANNEL_1, 0);
                set_pwm(g_tim10, TIM_CHANNEL_1, duty);
            }
            break;

        case MOTOR_FR:
            if (forward) {
                set_pwm(g_tim13, TIM_CHANNEL_1, 0);
                set_pwm(g_tim14, TIM_CHANNEL_1, duty);
            } else {
                set_pwm(g_tim14, TIM_CHANNEL_1, 0);
                set_pwm(g_tim13, TIM_CHANNEL_1, duty);
            }
            break;

        case MOTOR_RL:
            if (forward) {
                set_pwm(g_tim12, TIM_CHANNEL_2, 0);
                set_pwm(g_tim12, TIM_CHANNEL_1, duty);
            } else {
                set_pwm(g_tim12, TIM_CHANNEL_1, 0);
                set_pwm(g_tim12, TIM_CHANNEL_2, duty);
            }
            break;

        case MOTOR_RR:
            if (forward) {
                set_pwm(g_tim3, TIM_CHANNEL_1, 0);
                set_pwm(g_tim1, TIM_CHANNEL_1, duty);
            } else {
                set_pwm(g_tim1, TIM_CHANNEL_1, 0);
                set_pwm(g_tim3, TIM_CHANNEL_1, duty);
            }
            break;

        default: break;
    }
}

/* ── Public API ──────────────────────────────────────────────────── */

void Motor_Init(TIM_HandleTypeDef *htim10,
                TIM_HandleTypeDef *htim11,
                TIM_HandleTypeDef *htim12,
                TIM_HandleTypeDef *htim13,
                TIM_HandleTypeDef *htim14,
                TIM_HandleTypeDef *htim1,
                TIM_HandleTypeDef *htim3)
{
    g_tim10 = htim10;
    g_tim11 = htim11;
    g_tim12 = htim12;
    g_tim13 = htim13;
    g_tim14 = htim14;
    g_tim1  = htim1;
    g_tim3  = htim3;

    /* Start all PWM channels */
    HAL_TIM_PWM_Start(g_tim10, TIM_CHANNEL_1); /* FL LPWM */
    HAL_TIM_PWM_Start(g_tim11, TIM_CHANNEL_1); /* FL RPWM */
    HAL_TIM_PWM_Start(g_tim12, TIM_CHANNEL_1); /* RL RPWM */
    HAL_TIM_PWM_Start(g_tim12, TIM_CHANNEL_2); /* RL LPWM */
    HAL_TIM_PWM_Start(g_tim13, TIM_CHANNEL_1); /* FR LPWM */
    HAL_TIM_PWM_Start(g_tim14, TIM_CHANNEL_1); /* FR RPWM */
    HAL_TIM_PWM_Start(g_tim1,  TIM_CHANNEL_1); /* RR RPWM */
    HAL_TIM_PWM_Start(g_tim3,  TIM_CHANNEL_1); /* RR LPWM */

    memset(g_current_duty, 0, sizeof(g_current_duty));
    memset(g_dir,          0, sizeof(g_dir));
    g_ready = 1;
    Motor_StopAll();
}

void Motor_Set(Motor_ID id, float vel_mmps)
{
    if (!g_ready || (uint8_t)id >= MOTOR_NUM) return;

    /* Clamp velocity */
    if (vel_mmps >  MOTOR_MAX_SPEED_MMPS) vel_mmps =  MOTOR_MAX_SPEED_MMPS;
    if (vel_mmps < -MOTOR_MAX_SPEED_MMPS) vel_mmps = -MOTOR_MAX_SPEED_MMPS;

    uint8_t fwd = (vel_mmps >= 0.0f) ? 1u : 0u;

    /* Convert speed to duty cycle */
    uint32_t duty = (uint32_t)(fabsf(vel_mmps) / MOTOR_MAX_SPEED_MMPS
                               * (float)MOTOR_MAX_DUTY);

    g_dir[id] = (vel_mmps > 0.0f) ? 1 :
                (vel_mmps < 0.0f) ? -1 : 0;

    set_motor_raw(id, duty, fwd);
}

void Motor_StopAll(void)
{
    if (!g_ready) return;
    set_pwm(g_tim10, TIM_CHANNEL_1, 0);
    set_pwm(g_tim11, TIM_CHANNEL_1, 0);
    set_pwm(g_tim12, TIM_CHANNEL_1, 0);
    set_pwm(g_tim12, TIM_CHANNEL_2, 0);
    set_pwm(g_tim13, TIM_CHANNEL_1, 0);
    set_pwm(g_tim14, TIM_CHANNEL_1, 0);
    set_pwm(g_tim1,  TIM_CHANNEL_1, 0);
    set_pwm(g_tim3,  TIM_CHANNEL_1, 0);
    memset(g_current_duty, 0, sizeof(g_current_duty));
    memset(g_dir, 0, sizeof(g_dir));

    uint8_t dirs[MOTOR_NUM] = {0,0,0,0};
    Encoder_SendDirections(dirs);
}

int8_t Motor_GetDirection(Motor_ID id)
{
    if ((uint8_t)id >= MOTOR_NUM) return 0;
    return g_dir[id];
}
