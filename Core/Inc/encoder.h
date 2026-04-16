/**
 * encoder.h  —  Quadrature encoder driver  (4× RMCS-3070 motors)
 * Board : STM32F746G-DISCO
 *
 * ── Pin assignment ────────────────────────────────────────────────────
 *
 *  Motor        │ Timer │ CH_A (board label)    │ CH_B (board label)
 *  ──────────── │ ───── │ ───────────────────── │ ──────────────────────
 *  Front-Left   │ TIM5  │ A0   (PA0)            │ A1   (PA1)
 *  Front-Right  │ TIM2  │ D9   (PA15)           │ Morpho CN2 pin15 (PB3)
 *  Rear-Left    │ TIM4  │ Morpho CN2 pin13 (PB6)│ Morpho CN2 pin11 (PB7)
 *  Rear-Right   │ TIM3  │ D3   (PB4)            │ Morpho CN2 (PB5)
 *
 * ── CubeMX settings for all four timers ─────────────────────────────
 *   Combined Channels : Encoder Mode TI1 and TI2
 *   Counter Period    : 65535
 *   Prescaler         : 0
 *   CH1/CH2 Polarity  : Rising Edge
 *   Input Filter      : 4  (noise rejection)
 *
 * ── RMCS-3070 specs ─────────────────────────────────────────────────
 *   100 RPM output shaft @ 12 V
 *   Encoder: Hall effect, 11 PPR (before gearbox)
 *   Gear ratio: ~51.45  →  ticks/rev = 11 × 51.45 × 4 = 2264
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Encoder indices ──────────────────────────────────────────────── */
#define ENC_FL     0   /* TIM5 : A0 (PA0=CH1)  + A1 (PA1=CH2)           */
#define ENC_FR     1   /* TIM2 : D9 (PA15=CH1) + CN2p15 (PB3=CH2)       */
#define ENC_RL     2   /* TIM4 : CN2p13 (PB6=CH1) + CN2p11 (PB7=CH2)   */
#define ENC_RR     3   /* TIM3 : D3 (PB4=CH1)  + CN2 (PB5=CH2)          */
#define ENC_NUM    4

/* Backward-compatibility aliases (used by lcd_display.c) */
#define ENC_LEFT   ENC_FL
#define ENC_RIGHT  ENC_FR

/* ── Physical constants ─────────────────────────────────────────────── */
#define ENC_WHEEL_DIAM_MM    8.5f
#define ENC_WHEEL_CIRC_MM    26.70f
#define ENC_PPR_MOTOR        11
#define ENC_GEAR_RATIO       51.45f
#define ENC_TICKS_PER_REV    2264u      /* 11 × 51.45 × 4 (quadrature) */
#define ENC_MM_PER_TICK      (ENC_WHEEL_CIRC_MM / (float)ENC_TICKS_PER_REV)

/* Wheelbase (left-to-right wheel centre distance) — measure your robot */
#define ENC_WHEELBASE_MM     200.0f

/* ── Per-encoder state ──────────────────────────────────────────────── */
typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t  ticks;      /* accumulated signed ticks   */
    uint16_t last_cnt;   /* previous TIM->CNT          */
    float    vel_mmps;   /* velocity  mm/s             */
    float    dist_mm;    /* total distance  mm         */
    uint32_t last_ms;
} Encoder_t;

extern Encoder_t g_enc[ENC_NUM];

/* ── API ────────────────────────────────────────────────────────────── */

/**
 * Call after MX_TIM5/TIM2/TIM4/TIM3_Init() in main.c.
 *   htim_fl → TIM5  (Front-Left,  A0+A1)
 *   htim_fr → TIM2  (Front-Right, D9+CN2p15)
 *   htim_rl → TIM4  (Rear-Left,   CN2p13+CN2p11)
 *   htim_rr → TIM3  (Rear-Right,  D3+CN2)
 */
void    Encoder_Init   (TIM_HandleTypeDef *htim_fl,
                        TIM_HandleTypeDef *htim_fr,
                        TIM_HandleTypeDef *htim_rl,
                        TIM_HandleTypeDef *htim_rr);
void    Encoder_Update (void);           /* call every 10 ms */
void    Encoder_Reset  (uint8_t idx);
float   Encoder_VelMmps(uint8_t idx);
float   Encoder_DistMm (uint8_t idx);
int32_t Encoder_Ticks  (uint8_t idx);

#endif /* ENCODER_H */
