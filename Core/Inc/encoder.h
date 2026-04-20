/**
 * encoder.h — Encoder driver reading from ESP32 co-processor via I2C
 * STM32F746G-DISCO
 *
 * ── Architecture ─────────────────────────────────────────────────────
 *   ESP32-WROOM acts as I2C slave (addr 0x30).
 *   It runs hardware PCNT quadrature counting on all 4 encoders.
 *   STM32 reads tick counts and velocities every 20ms via I2C1.
 *
 * ── I2C Protocol ─────────────────────────────────────────────────────
 *   STM32 READS 32 bytes from ESP32 (addr 0x30):
 *     Bytes  0-3  : FL ticks  (int32,  little-endian)
 *     Bytes  4-7  : FR ticks  (int32,  little-endian)
 *     Bytes  8-11 : RL ticks  (int32,  little-endian)
 *     Bytes 12-15 : RR ticks  (int32,  little-endian)
 *     Bytes 16-19 : FL speed  (float,  mm/s, little-endian)
 *     Bytes 20-23 : FR speed  (float,  mm/s, little-endian)
 *     Bytes 24-27 : RL speed  (float,  mm/s, little-endian)
 *     Bytes 28-31 : RR speed  (float,  mm/s, little-endian)
 *
 *   STM32 WRITES 4 bytes to ESP32 (addr 0x30):
 *     Byte 0 : FL direction (0=forward, 1=reverse)
 *     Byte 1 : FR direction
 *     Byte 2 : RL direction
 *     Byte 3 : RR direction
 *
 * ── I2C Bus ──────────────────────────────────────────────────────────
 *   PB8 (D15/CN2-pin3) = SCL   shared with ICM-20948 (0x68)
 *   PB9 (D14/CN2-pin1) = SDA   pull-ups: 4.7kΩ to 3.3V on each line
 *
 * ── Physical constants (RMCS-3070 with quadrature ×4) ────────────────
 *   Ticks per wheel rev = 11 × 51.45 × 4 = 2264
 *   Wheel circumference = 26.70 mm
 *   mm per tick         = 0.01179 mm
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── ESP32 I2C address ───────────────────────────────────────────── */
#define ESP32_ENCODER_ADDR   (0x30 << 1)   /* HAL uses 8-bit addr    */
#define ESP32_READ_LEN       32
#define ESP32_WRITE_LEN      4

/* ── Encoder indices ─────────────────────────────────────────────── */
#define ENC_FL     0
#define ENC_FR     1
#define ENC_RL     2
#define ENC_RR     3
#define ENC_NUM    4

/* Aliases for side-based access */
#define ENC_LEFT   ENC_FL
#define ENC_RIGHT  ENC_FR

/* ── Physical constants ─────────────────────────────────────────── */
#define ENC_TICKS_PER_REV    2264u      /* quadrature ×4             */
#define ENC_WHEEL_CIRC_MM    26.70f
#define ENC_MM_PER_TICK      (ENC_WHEEL_CIRC_MM / (float)ENC_TICKS_PER_REV)
#define ENC_WHEELBASE_MM     200.0f

/* ── Per-encoder state (cached from ESP32) ───────────────────────── */
typedef struct {
    int32_t  ticks;       /* total accumulated ticks                  */
    float    vel_mmps;    /* velocity mm/s  (+ve=fwd, -ve=rev)        */
    float    dist_mm;     /* total distance mm                        */
    uint8_t  direction;   /* 0=forward 1=reverse                      */
} Encoder_t;

extern Encoder_t g_enc[ENC_NUM];

/* ── API ─────────────────────────────────────────────────────────── */

/**
 * @brief Init encoder module. Call after MX_I2C1_Init().
 * @param hi2c  I2C1 handle (shared with ICM-20948)
 */
void Encoder_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read latest data from ESP32 over I2C.
 *        Call every 20ms (50Hz) from main loop.
 * @return HAL_OK on success, HAL_ERROR if ESP32 not responding
 */
HAL_StatusTypeDef Encoder_Update(void);

/**
 * @brief Send direction info to ESP32.
 *        Called automatically by Motor_SetMotor().
 * @param directions  Array of 4 bytes: [FL,FR,RL,RR] 0=fwd 1=rev
 */
void Encoder_SendDirections(uint8_t directions[ENC_NUM]);

/**
 * @brief Set direction for a single motor.
 * @param enc_id   ENC_FL, ENC_FR, ENC_RL, or ENC_RR
 * @param forward  1=forward, 0=reverse
 */
void Encoder_SetDirection(uint8_t enc_id, uint8_t forward);

/* Getters */
float    Encoder_VelMmps (uint8_t idx);
float    Encoder_DistMm  (uint8_t idx);
int32_t  Encoder_Ticks   (uint8_t idx);
void     Encoder_Reset   (uint8_t idx);

#endif /* ENCODER_H */
