/**
 * motor.c — 4× BTS7960 individual motor driver (true 4WD)
 * STM32F746G-DISCO
 *
 * FL: TIM11(RPWM/PF7) + TIM10(LPWM/PF6)
 * FR: TIM14(RPWM/PF9) + TIM13(LPWM/PF8)
 * RL: TIM12_CH1(RPWM/PH6) + TIM12_CH2(LPWM/PB15)
 * RR: TIM2_CH1(RPWM/PA15) + TIM3_CH1(LPWM/PB4)
 *
 * Voltage protection:
 *   APB2 timers (TIM10/TIM11) max CCR = 9254
 *   APB1 timers (TIM2/TIM3/TIM12/TIM13/TIM14) max CCR = 4626
 *
 * Soft start: duty ramps by max 200 counts per 10ms cycle
 */

#include "motor.h"
#include "encoder.h"
#include <math.h>
#include <string.h>

/* ── Timer handles ───────────────────────────────────────────────── */
static TIM_HandleTypeDef *g_tim10  = NULL; /* FL LPWM  PF6 */
static TIM_HandleTypeDef *g_tim11  = NULL; /* FL RPWM  PF7 */
static TIM_HandleTypeDef *g_tim12  = NULL; /* RL RPWM+LPWM PH6+PB15 */
static TIM_HandleTypeDef *g_tim13  = NULL; /* FR LPWM  PF8 */
static TIM_HandleTypeDef *g_tim14  = NULL; /* FR RPWM  PF9 */
static TIM_HandleTypeDef *g_tim2   = NULL; /* RR RPWM  PA15 */
static TIM_HandleTypeDef *g_tim3   = NULL; /* RR LPWM  PB4 */

static uint8_t  g_ready = 0;
static int8_t   g_dir[MOTOR_NUM]          = {0};
static uint32_t g_current_duty[MOTOR_NUM] = {0};

/* ── Set PWM helper ──────────────────────────────────────────────── */
static inline void set_pwm(TIM_HandleTypeDef *htim,
                           uint32_t ch,
                           uint32_t duty)
{
    __HAL_TIM_SET_COMPARE(htim, ch, duty);
}

static inline uint32_t max_duty_for_motor(Motor_ID id)
{
    switch (id) {
        case MOTOR_FL:
            return MOTOR_MAX_DUTY_APB2; /* TIM10 + TIM11 */
        case MOTOR_FR:
        case MOTOR_RL:
        case MOTOR_RR:
        default:
            return MOTOR_MAX_DUTY_APB1; /* TIM14/TIM13/TIM12/TIM2/TIM3 */
    }
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
    uint32_t max_duty = max_duty_for_motor(id);

    /* Clamp to voltage-protected max */
    if (duty > max_duty) duty = max_duty;

    /* Apply soft start ramp */
    duty = ramp_duty(g_current_duty[id], duty);
    g_current_duty[id] = duty;

    /* Update encoder direction */
    Encoder_SetDirection((uint8_t)id, forward);

    switch (id) {
        case MOTOR_FL:
            /* Never drive both RPWM and LPWM together */
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
                set_pwm(g_tim2, TIM_CHANNEL_1, duty);
            } else {
                set_pwm(g_tim2, TIM_CHANNEL_1, 0);
                set_pwm(g_tim3, TIM_CHANNEL_1, duty);
            }
            break;

        default:
            break;
    }
}

/* ── Public API ──────────────────────────────────────────────────── */

void Motor_Init(TIM_HandleTypeDef *htim10,
                TIM_HandleTypeDef *htim11,
                TIM_HandleTypeDef *htim12,
                TIM_HandleTypeDef *htim13,
                TIM_HandleTypeDef *htim14,
                TIM_HandleTypeDef *htim2,
                TIM_HandleTypeDef *htim3)
{
    g_tim10 = htim10;
    g_tim11 = htim11;
    g_tim12 = htim12;
    g_tim13 = htim13;
    g_tim14 = htim14;
    g_tim2  = htim2;
    g_tim3  = htim3;

    /* Start all PWM channels */
    HAL_TIM_PWM_Start(g_tim10, TIM_CHANNEL_1); /* FL LPWM */
    HAL_TIM_PWM_Start(g_tim11, TIM_CHANNEL_1); /* FL RPWM */
    HAL_TIM_PWM_Start(g_tim12, TIM_CHANNEL_1); /* RL RPWM */
    HAL_TIM_PWM_Start(g_tim12, TIM_CHANNEL_2); /* RL LPWM */
    HAL_TIM_PWM_Start(g_tim13, TIM_CHANNEL_1); /* FR LPWM */
    HAL_TIM_PWM_Start(g_tim14, TIM_CHANNEL_1); /* FR RPWM */
    HAL_TIM_PWM_Start(g_tim2,  TIM_CHANNEL_1); /* RR RPWM */
    HAL_TIM_PWM_Start(g_tim3,  TIM_CHANNEL_1); /* RR LPWM */

    memset(g_current_duty, 0, sizeof(g_current_duty));
    memset(g_dir,          0, sizeof(g_dir));
    g_ready = 1;
    Motor_StopAll();
}

void Motor_Set(Motor_ID id, float vel_mmps)
{
    uint32_t max_duty;
    float abs_vel;
    uint8_t fwd;
    uint32_t duty;

    if (!g_ready || (uint8_t)id >= MOTOR_NUM) return;

    /* Clamp velocity */
    if (vel_mmps >  MOTOR_MAX_SPEED_MMPS) vel_mmps =  MOTOR_MAX_SPEED_MMPS;
    if (vel_mmps < -MOTOR_MAX_SPEED_MMPS) vel_mmps = -MOTOR_MAX_SPEED_MMPS;

    fwd = (vel_mmps >= 0.0f) ? 1u : 0u;
    abs_vel = fabsf(vel_mmps);
    max_duty = max_duty_for_motor(id);

    /* Convert speed to duty cycle using the motor-specific safe max */
    duty = (uint32_t)((abs_vel / MOTOR_MAX_SPEED_MMPS) * (float)max_duty);

    g_dir[id] = (vel_mmps > 0.0f) ? 1 :
                (vel_mmps < 0.0f) ? -1 : 0;

    set_motor_raw(id, duty, fwd);
}

void Motor_StopAll(void)
{
    uint8_t dirs[MOTOR_NUM] = {0, 0, 0, 0};

    if (!g_ready) return;

    set_pwm(g_tim10, TIM_CHANNEL_1, 0);
    set_pwm(g_tim11, TIM_CHANNEL_1, 0);
    set_pwm(g_tim12, TIM_CHANNEL_1, 0);
    set_pwm(g_tim12, TIM_CHANNEL_2, 0);
    set_pwm(g_tim13, TIM_CHANNEL_1, 0);
    set_pwm(g_tim14, TIM_CHANNEL_1, 0);
    set_pwm(g_tim2,  TIM_CHANNEL_1, 0);
    set_pwm(g_tim3,  TIM_CHANNEL_1, 0);

    memset(g_current_duty, 0, sizeof(g_current_duty));
    memset(g_dir, 0, sizeof(g_dir));

    Encoder_SendDirections(dirs);
}

int8_t Motor_GetDirection(Motor_ID id)
{
    if ((uint8_t)id >= MOTOR_NUM) return 0;
    return g_dir[id];
}
