/**
 * motor.c — 4-motor PWM velocity driver
 * STM32F746G-DISCO | TIM1 CH1-CH4 (20 kHz) + 4× direction GPIO
 */
#include "motor.h"
#include <math.h>

/* ── Module state ───────────────────────────────────────────────────── */
static TIM_HandleTypeDef *g_htim = NULL;

/* ── Lookup tables indexed by Motor_ID ─────────────────────────────── */
static const uint32_t MOTOR_CH[MOTOR_NUM] = {
    MOTOR_FL_TIM_CH,   /* MOTOR_FL */
    MOTOR_FR_TIM_CH,   /* MOTOR_FR */
    MOTOR_RL_TIM_CH,   /* MOTOR_RL */
    MOTOR_RR_TIM_CH,   /* MOTOR_RR */
};

static GPIO_TypeDef * const MOTOR_DIR_PORT[MOTOR_NUM] = {
    MOTOR_FL_DIR_PORT,
    MOTOR_FR_DIR_PORT,
    MOTOR_RL_DIR_PORT,
    MOTOR_RR_DIR_PORT,
};

static const uint16_t MOTOR_DIR_PIN[MOTOR_NUM] = {
    MOTOR_FL_DIR_PIN,
    MOTOR_FR_DIR_PIN,
    MOTOR_RL_DIR_PIN,
    MOTOR_RR_DIR_PIN,
};

/*
 * Polarity: +1 = positive velocity → DIR pin LOW  (forward)
 *           -1 = positive velocity → DIR pin HIGH (right-side mirrored mount)
 *
 * Right-side motors are physically mirror-mounted, so they need their
 * direction inverted to produce the same forward motion.
 * Change -1 to +1 if your wiring handles reversal at the H-bridge.
 */
static const int8_t MOTOR_POLARITY[MOTOR_NUM] = {
    +1,   /* FL: standard orientation */
    -1,   /* FR: mirrored — invert    */
    +1,   /* RL: standard orientation */
    -1,   /* RR: mirrored — invert    */
};

/* ── Public API ─────────────────────────────────────────────────────── */

void Motor_Init(TIM_HandleTypeDef *htim)
{
    g_htim = htim;
    for (int i = 0; i < MOTOR_NUM; i++) {
        HAL_GPIO_WritePin(MOTOR_DIR_PORT[i], MOTOR_DIR_PIN[i], GPIO_PIN_RESET);
        HAL_TIM_PWM_Start(g_htim, MOTOR_CH[i]);
        __HAL_TIM_SET_COMPARE(g_htim, MOTOR_CH[i], 0);
    }
}

void Motor_Set(Motor_ID id, float vel_mmps)
{
    if ((uint8_t)id >= MOTOR_NUM || g_htim == NULL) return;

    /* Apply polarity for mirrored motors */
    float v = vel_mmps * (float)MOTOR_POLARITY[id];

    /* Clamp to physical limit */
    if (v >  MOTOR_MAX_SPEED_MMPS) v =  MOTOR_MAX_SPEED_MMPS;
    if (v < -MOTOR_MAX_SPEED_MMPS) v = -MOTOR_MAX_SPEED_MMPS;

    /* Direction pin: LOW = forward, HIGH = reverse */
    HAL_GPIO_WritePin(MOTOR_DIR_PORT[id], MOTOR_DIR_PIN[id],
                      (v >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* PWM duty proportional to |velocity| */
    uint32_t duty = (uint32_t)(fabsf(v) / MOTOR_MAX_SPEED_MMPS
                                * (float)MOTOR_PWM_ARR);
    if (duty > MOTOR_PWM_ARR) duty = MOTOR_PWM_ARR;

    __HAL_TIM_SET_COMPARE(g_htim, MOTOR_CH[id], duty);
}

void Motor_StopAll(void)
{
    if (g_htim == NULL) return;
    for (int i = 0; i < MOTOR_NUM; i++)
        __HAL_TIM_SET_COMPARE(g_htim, MOTOR_CH[i], 0);
}
