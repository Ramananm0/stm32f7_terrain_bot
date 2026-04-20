/**
 * encoder.c — I2C encoder reader from ESP32 co-processor
 * STM32F746G-DISCO
 *
 * Reads 32 bytes from ESP32 slave (0x30) every 20ms.
 * Sends 4 direction bytes to ESP32 when motor direction changes.
 */

#include "encoder.h"
#include <string.h>
#include <math.h>

Encoder_t g_enc[ENC_NUM];

static I2C_HandleTypeDef *g_hi2c     = NULL;
static uint8_t            g_dirs[ENC_NUM] = {0};
static uint8_t            g_dirs_dirty    = 0;
static uint8_t            g_initialised   = 0;

/* ── Init ────────────────────────────────────────────────────────── */
void Encoder_Init(I2C_HandleTypeDef *hi2c)
{
    g_hi2c = hi2c;
    memset(g_enc,  0, sizeof(g_enc));
    memset(g_dirs, 0, sizeof(g_dirs));
    g_dirs_dirty  = 1;
    g_initialised = 1;
}

/* ── Read from ESP32 ─────────────────────────────────────────────── */
HAL_StatusTypeDef Encoder_Update(void)
{
    if (!g_initialised || g_hi2c == NULL) return HAL_ERROR;

    /* Send direction update if needed */
    if (g_dirs_dirty) {
        HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(
            g_hi2c, ESP32_ENCODER_ADDR,
            g_dirs, ESP32_WRITE_LEN, 10);
        if (s == HAL_OK) g_dirs_dirty = 0;
    }

    /* Read 32 bytes from ESP32 */
    uint8_t buf[ESP32_READ_LEN];
    HAL_StatusTypeDef s = HAL_I2C_Master_Receive(
        g_hi2c, ESP32_ENCODER_ADDR,
        buf, ESP32_READ_LEN, 20);

    if (s != HAL_OK) return s;

    /* Parse ticks [0-15] */
    for (int i = 0; i < ENC_NUM; i++) {
        int32_t ticks;
        memcpy(&ticks, &buf[i * 4], 4);
        float prev_dist = g_enc[i].dist_mm;
        int32_t prev_ticks = g_enc[i].ticks;
        g_enc[i].ticks   = ticks;
        g_enc[i].dist_mm = (float)ticks * ENC_MM_PER_TICK;
        (void)prev_dist;
        (void)prev_ticks;
    }

    /* Parse speeds [16-31] */
    for (int i = 0; i < ENC_NUM; i++) {
        float vel;
        memcpy(&vel, &buf[16 + i * 4], 4);
        g_enc[i].vel_mmps = vel;
    }

    return HAL_OK;
}

/* ── Direction management ────────────────────────────────────────── */
void Encoder_SetDirection(uint8_t enc_id, uint8_t forward)
{
    if (enc_id >= ENC_NUM) return;
    uint8_t dir = forward ? 0 : 1;
    if (g_dirs[enc_id] != dir) {
        g_dirs[enc_id] = dir;
        g_enc[enc_id].direction = dir;
        g_dirs_dirty = 1;
    }
}

void Encoder_SendDirections(uint8_t directions[ENC_NUM])
{
    memcpy(g_dirs, directions, ENC_NUM);
    g_dirs_dirty = 1;
}

/* ── Getters ─────────────────────────────────────────────────────── */
float   Encoder_VelMmps(uint8_t i) { return i < ENC_NUM ? g_enc[i].vel_mmps : 0.0f; }
float   Encoder_DistMm (uint8_t i) { return i < ENC_NUM ? g_enc[i].dist_mm  : 0.0f; }
int32_t Encoder_Ticks  (uint8_t i) { return i < ENC_NUM ? g_enc[i].ticks    : 0;    }

void Encoder_Reset(uint8_t i)
{
    if (i >= ENC_NUM) return;
    g_enc[i].ticks    = 0;
    g_enc[i].dist_mm  = 0.0f;
    g_enc[i].vel_mmps = 0.0f;
}
