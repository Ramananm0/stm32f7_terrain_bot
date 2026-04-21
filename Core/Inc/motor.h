/**
 * motor.h — 4× BTS7960 individual motor driver (true 4WD)
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

typedef enum {
    MOTOR_FL  = 0,
    MOTOR_FR  = 1,
    MOTOR_RL  = 2,
    MOTOR_RR  = 3,
    MOTOR_NUM = 4
} Motor_ID;

#define MOTOR_MAX_SPEED_MMPS   44.5f

#define MOTOR_PWM_ARR_APB2     10799u
#define MOTOR_PWM_ARR_APB1      5399u

#define MOTOR_BATTERY_MAX_V    14.0f
#define MOTOR_RATED_V          12.0f

#define MOTOR_MAX_DUTY_APB2    9254u
#define MOTOR_MAX_DUTY_APB1    4626u

#define MOTOR_RAMP_STEP        200u
#define MOTOR_ILIMIT           (MOTOR_MAX_SPEED_MMPS * 0.5f)

void Motor_Init(TIM_HandleTypeDef *htim10,
                TIM_HandleTypeDef *htim11,
                TIM_HandleTypeDef *htim12,
                TIM_HandleTypeDef *htim13,
                TIM_HandleTypeDef *htim14,
                TIM_HandleTypeDef *htim2,
                TIM_HandleTypeDef *htim3);

void Motor_Set(Motor_ID id, float vel_mmps);
void Motor_StopAll(void);
int8_t Motor_GetDirection(Motor_ID id);
uint32_t Motor_GetDuty(Motor_ID id);

#endif
