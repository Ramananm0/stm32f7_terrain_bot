/**
 * encoder.c — Single-pin interrupt speed encoder
 * STM32F746G-DISCO
 *
 * FL → A0  (PA0)  EXTI0
 * FR → D3  (PB4)  EXTI4
 * RL → D9  (PA15) EXTI15
 * RR → D2  (PG6)  EXTI6
 */

#include "encoder.h"
#include <string.h>

Encoder_t g_enc[ENC_NUM];

/* ── Init ────────────────────────────────────────────────────────── */
void Encoder_Init(void)
{
    memset(g_enc, 0, sizeof(g_enc));
    for (int i = 0; i < ENC_NUM; i++) {
        g_enc[i].direction = 1;   /* default forward */
        g_enc[i].last_ms   = HAL_GetTick();
    }
}

/* ── Called from EXTI ISR ────────────────────────────────────────── */
void Encoder_PulseISR(uint8_t enc_id)
{
    if (enc_id >= ENC_NUM) return;
    g_enc[enc_id].pulse_count++;
    g_enc[enc_id].total_ticks++;
}

/* ── Set direction from motor command ────────────────────────────── */
void Encoder_SetDirection(uint8_t enc_id, uint8_t forward)
{
    if (enc_id >= ENC_NUM) return;
    g_enc[enc_id].direction = forward ? 1 : -1;
}

/* ── Update velocities — call every 100ms ────────────────────────── */
void Encoder_Update(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < ENC_NUM; i++) {
        uint32_t dt_ms = now - g_enc[i].last_ms;
        if (dt_ms == 0) continue;

        /* Snapshot pulse count atomically */
        __disable_irq();
        uint32_t pulses = g_enc[i].pulse_count;
        g_enc[i].pulse_count = 0;
        __enable_irq();

        /* Speed = pulses × mm_per_tick / dt_seconds */
        float dt_s = (float)dt_ms * 0.001f;
        float speed = (float)pulses * ENC_MM_PER_TICK / dt_s;

        /* Apply direction */
        g_enc[i].vel_mmps = speed * (float)g_enc[i].direction;

        /* Accumulate distance */
        g_enc[i].dist_mm += (float)pulses * ENC_MM_PER_TICK
                            * (float)g_enc[i].direction;

        g_enc[i].last_ms = now;
    }
}

/* ── Getters ─────────────────────────────────────────────────────── */
float    Encoder_VelMmps(uint8_t i) { return i < ENC_NUM ? g_enc[i].vel_mmps    : 0.0f; }
float    Encoder_DistMm (uint8_t i) { return i < ENC_NUM ? g_enc[i].dist_mm     : 0.0f; }
uint32_t Encoder_Ticks  (uint8_t i) { return i < ENC_NUM ? g_enc[i].total_ticks : 0;    }

void Encoder_Reset(uint8_t i)
{
    if (i >= ENC_NUM) return;
    __disable_irq();
    g_enc[i].pulse_count  = 0;
    g_enc[i].total_ticks  = 0;
    g_enc[i].vel_mmps     = 0.0f;
    g_enc[i].dist_mm      = 0.0f;
    __enable_irq();
}
