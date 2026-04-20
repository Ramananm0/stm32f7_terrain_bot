/**
 * encoder.h — Single-pin speed encoder driver
 * STM32F746G-DISCO (Arduino header only)
 *
 * Uses GPIO interrupt (EXTI) on CH_A signal only.
 * Speed calculated from pulse count per time interval.
 * Direction known from BTS7960 command, not encoder.
 *
 * ── Pin Assignment (all Arduino header) ─────────────────────────────
 *   FL encoder CH_A → A0  (PA0)  EXTI0
 *   FR encoder CH_A → D3  (PB4)  EXTI4
 *   RL encoder CH_A → D9  (PA15) EXTI15
 *   RR encoder CH_A → D10 (PA8)  EXTI8 — wait PA8=TIM1_CH1
 *
 * Actually safe assignment:
 *   FL → A0  (PA0)  EXTI0
 *   FR → D3  (PB4)  EXTI4
 *   RL → D9  (PA15) EXTI15
 *   RR → D2  (PG6)  EXTI6
 *
 * ── RMCS-3070 specs ─────────────────────────────────────────────────
 *   Encoder: Hall effect, 11 PPR (motor shaft)
 *   Gear ratio: ~51.45
 *   Ticks per wheel rev = 11 × 51.45 = 566 (single channel)
 *   Wheel diameter: 8.5 mm → circumference = 26.70 mm
 *   mm per tick = 26.70 / 566 = 0.04717 mm
 *
 * ── CubeMX settings per pin ─────────────────────────────────────────
 *   Mode     : GPIO_EXTI (External Interrupt)
 *   Trigger  : Rising Edge
 *   Pull     : Pull-Down
 *   Enable NVIC for each EXTI line
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── Encoder indices ─────────────────────────────────────────────── */
#define ENC_FL    0   /* A0  (PA0)  EXTI0  */
#define ENC_FR    1   /* D3  (PB4)  EXTI4  */
#define ENC_RL    2   /* D9  (PA15) EXTI15 */
#define ENC_RR    3   /* D2  (PG6)  EXTI6  */
#define ENC_NUM   4

/* Aliases for left/right side */
#define ENC_LEFT   ENC_FL
#define ENC_RIGHT  ENC_FR

/* ── Physical constants ─────────────────────────────────────────── */
#define ENC_PPR_MOTOR       11        /* pulses per motor revolution  */
#define ENC_GEAR_RATIO      51.45f    /* gearbox ratio                */
#define ENC_TICKS_PER_REV   566u      /* 11 × 51.45 (single channel)  */
#define ENC_WHEEL_CIRC_MM   26.70f    /* π × 8.5 mm wheel diameter    */
#define ENC_MM_PER_TICK     (ENC_WHEEL_CIRC_MM / (float)ENC_TICKS_PER_REV)
#define ENC_WHEELBASE_MM    200.0f    /* left-to-right wheel centres  */

/* ── Per-encoder state ───────────────────────────────────────────── */
typedef struct {
    volatile uint32_t pulse_count;   /* pulses since last update     */
    volatile uint32_t total_ticks;   /* total pulses ever            */
    float    vel_mmps;               /* calculated speed mm/s        */
    float    dist_mm;                /* total distance mm            */
    uint32_t last_ms;                /* timestamp of last update     */
    int8_t   direction;             /* +1 forward, -1 reverse       */
} Encoder_t;

extern Encoder_t g_enc[ENC_NUM];

/* ── API ─────────────────────────────────────────────────────────── */

/**
 * @brief Init encoder state. Call before main loop.
 *        CubeMX must configure EXTI pins before this.
 */
void Encoder_Init(void);

/**
 * @brief Update velocities. Call every 100ms from main loop.
 */
void Encoder_Update(void);

/**
 * @brief Call from EXTI ISR for each encoder pin.
 * @param enc_id  ENC_FL, ENC_FR, ENC_RL, or ENC_RR
 */
void Encoder_PulseISR(uint8_t enc_id);

/**
 * @brief Set direction for an encoder (from motor command).
 * @param enc_id    Encoder index
 * @param forward   1=forward, 0=reverse
 */
void Encoder_SetDirection(uint8_t enc_id, uint8_t forward);

/* Getters */
float    Encoder_VelMmps (uint8_t idx);
float    Encoder_DistMm  (uint8_t idx);
uint32_t Encoder_Ticks   (uint8_t idx);
void     Encoder_Reset   (uint8_t idx);

#endif /* ENCODER_H */
