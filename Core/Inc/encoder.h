/**
 * encoder.h  —  Quadrature encoder driver  (RMCS-3070 motors)
 * Board : STM32F746G-DISCO
 *
 * ── Pin assignment using ARDUINO LABELS on the board ────────────────
 *
 *  Side   │ Timer │ Arduino CH_A │ Arduino CH_B │ STM32 (CubeMX)
 *  ─────  │ ───── │ ────────────  │ ────────────  │ ──────────────────
 *  LEFT   │ TIM5  │ A0           │ A1           │ PA0 CH1 / PA1 CH2
 *  RIGHT  │ TIM2  │ D9           │ CN2 pin 15   │ PA15 CH1 / PB3 CH2
 *
 *  LEFT  encoder = represents FL + RL wheels (left side of robot)
 *  RIGHT encoder = represents FR + RR wheels (right side of robot)
 *
 *  This is sufficient for differential drive SLAM odometry.
 *  If 4 individual encoders are needed later, add TIM3 and TIM4
 *  using the Morpho connector (CN1/CN2) on the board.
 *
 * ── CubeMX settings for both timers ────────────────────────────────
 *   Combined Channels : Encoder Mode TI1 and TI2
 *   Counter Period    : 65535
 *   Prescaler         : 0
 *   CH1/CH2 Polarity  : Rising Edge
 *   Input Filter      : 4  (noise rejection)
 *
 * ── RMCS-3070 specs ─────────────────────────────────────────────────
 *   100 RPM @ 12V
 *   Encoder: Hall effect, 11 PPR before gearbox
 *   Gear ratio: ~51.45  →  ticks/rev = 11 × 51.45 × 4 = 2264
 *
 * ── Wheel (user spec) ───────────────────────────────────────────────
 *   Diameter : 8.5 mm  →  circumference = 26.70 mm
 *   Width    : 3.8 mm
 *   mm/tick  : 26.70 / 2264 = 0.01179 mm
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

#define ENC_LEFT    0   /* TIM5 : A0 (PA0=CH1)  + A1 (PA1=CH2)  */
#define ENC_RIGHT   1   /* TIM2 : D9 (PA15=CH1) + CN2p15 (PB3=CH2) */
#define ENC_NUM     2

/* Physical constants */
#define ENC_WHEEL_DIAM_MM    8.5f
#define ENC_WHEEL_CIRC_MM    26.70f
#define ENC_PPR_MOTOR        11
#define ENC_GEAR_RATIO       51.45f
#define ENC_TICKS_PER_REV    2264u      /* 11 × 51.45 × 4 */
#define ENC_MM_PER_TICK      (ENC_WHEEL_CIRC_MM / (float)ENC_TICKS_PER_REV)

/* Wheelbase (left-to-right wheel distance) — measure your robot */
#define ENC_WHEELBASE_MM     200.0f

typedef struct {
    TIM_HandleTypeDef *htim;
    int32_t  ticks;         /* accumulated signed ticks   */
    uint16_t last_cnt;      /* previous TIM->CNT          */
    float    vel_mmps;      /* velocity  mm/s             */
    float    dist_mm;       /* total distance  mm         */
    uint32_t last_ms;
} Encoder_t;

extern Encoder_t g_enc[ENC_NUM];

/**
 * Call after MX_TIM5_Init() and MX_TIM2_Init() in main.c
 * htim5 → A0+A1 (LEFT)
 * htim2 → D9+CN2pin15 (RIGHT)
 */
void    Encoder_Init   (TIM_HandleTypeDef *htim5,
                        TIM_HandleTypeDef *htim2);
void    Encoder_Update (void);           /* call every 10 ms */
void    Encoder_Reset  (uint8_t idx);
float   Encoder_VelMmps(uint8_t idx);
float   Encoder_DistMm (uint8_t idx);
int32_t Encoder_Ticks  (uint8_t idx);

#endif /* ENCODER_H */
