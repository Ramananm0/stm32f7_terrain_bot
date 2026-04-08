/**
 * encoder.c  —  Quadrature encoder driver
 * STM32F746G-DISCO  |  TIM5 (A0+A1 = LEFT)  TIM2 (D9+CN2p15 = RIGHT)
 */
#include "encoder.h"
#include <string.h>

Encoder_t g_enc[ENC_NUM];

void Encoder_Init(TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim2)
{
    memset(g_enc, 0, sizeof(g_enc));

    g_enc[ENC_LEFT].htim  = htim5;   /* A0 (PA0=CH1) + A1 (PA1=CH2) */
    g_enc[ENC_RIGHT].htim = htim2;   /* D9 (PA15=CH1) + CN2p15 (PB3=CH2) */

    for (int i = 0; i < ENC_NUM; i++) {
        HAL_TIM_Encoder_Start(g_enc[i].htim, TIM_CHANNEL_ALL);
        g_enc[i].last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(g_enc[i].htim);
        g_enc[i].last_ms  = HAL_GetTick();
    }
}

void Encoder_Update(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < ENC_NUM; i++) {
        uint16_t cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(g_enc[i].htim);
        int16_t  delta = (int16_t)(cnt - g_enc[i].last_cnt);
        g_enc[i].last_cnt = cnt;
        g_enc[i].ticks   += delta;

        uint32_t dt_ms = now - g_enc[i].last_ms;
        if (dt_ms > 0) {
            g_enc[i].vel_mmps = (float)delta * ENC_MM_PER_TICK
                                 / ((float)dt_ms * 0.001f);
            g_enc[i].last_ms  = now;
        }
        g_enc[i].dist_mm += (float)delta * ENC_MM_PER_TICK;
    }
}

void Encoder_Reset(uint8_t i)
{
    if (i >= ENC_NUM) return;
    g_enc[i].ticks    = 0;
    g_enc[i].dist_mm  = 0.0f;
    g_enc[i].vel_mmps = 0.0f;
    __HAL_TIM_SET_COUNTER(g_enc[i].htim, 0);
    g_enc[i].last_cnt = 0;
}

float   Encoder_VelMmps(uint8_t i) { return i < ENC_NUM ? g_enc[i].vel_mmps : 0.0f; }
float   Encoder_DistMm (uint8_t i) { return i < ENC_NUM ? g_enc[i].dist_mm  : 0.0f; }
int32_t Encoder_Ticks  (uint8_t i) { return i < ENC_NUM ? g_enc[i].ticks    : 0;    }
