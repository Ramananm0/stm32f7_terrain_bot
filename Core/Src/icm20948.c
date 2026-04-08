/**
 * icm20948.c  —  ICM-20948 driver implementation
 * STM32F746G-DISCO  |  I2C3  |  HAL
 */
#include "icm20948.h"
#include <math.h>
#include <string.h>

#define I2C_TO  10   /* ms */

/* ── Low-level helpers ───────────────────────────────────────────── */

static HAL_StatusTypeDef bank(I2C_HandleTypeDef *h, uint8_t b)
{
    uint8_t buf[2] = {REG_BANK_SEL, b};
    return HAL_I2C_Master_Transmit(h, ICM20948_ADDR, buf, 2, I2C_TO);
}

static HAL_StatusTypeDef wr(I2C_HandleTypeDef *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(h, ICM20948_ADDR, buf, 2, I2C_TO);
}

static HAL_StatusTypeDef rd(I2C_HandleTypeDef *h, uint8_t reg,
                             uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(h, ICM20948_ADDR, &reg, 1, I2C_TO);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(h, ICM20948_ADDR, buf, len, I2C_TO);
}

/* Write one register inside AK09916 via ICM I2C master */
static void ak_write(I2C_HandleTypeDef *h, uint8_t ak_reg, uint8_t val)
{
    bank(h, 0x30);                    /* Bank 3 */
    wr(h, B3_SLV0_ADDR, AK_ADDR);    /* write direction */
    wr(h, B3_SLV0_REG,  ak_reg);
    wr(h, B3_SLV0_DO,   val);
    wr(h, B3_SLV0_CTRL, 0x81);       /* enable, 1 byte */
    HAL_Delay(2);
    bank(h, 0x00);                    /* back to Bank 0 */
}

/* ── Init ────────────────────────────────────────────────────────── */

HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who;

    /* 1. Soft reset */
    bank(hi2c, 0x00);
    wr(hi2c, B0_PWR_MGMT_1, 0x80);   /* DEVICE_RESET */
    HAL_Delay(100);

    /* 2. Wake up, auto-select clock */
    wr(hi2c, B0_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* 3. Check WHO_AM_I */
    if (rd(hi2c, B0_WHO_AM_I, &who, 1) != HAL_OK || who != 0xEA)
        return HAL_ERROR;

    /* 4. Enable accel + gyro */
    wr(hi2c, B0_PWR_MGMT_2, 0x00);

    /* 5. Gyro config — Bank 2
     *    SMPLRT_DIV=10 → ODR = 1125/(1+10) ≈ 102 Hz
     *    CONFIG_1: DLPF=001(119Hz), FS=01(±500dps), FCHOICE=1 → 0x0B */
    bank(hi2c, 0x20);
    wr(hi2c, B2_GYRO_SMPLRT_DIV, 10);
    wr(hi2c, B2_GYRO_CONFIG_1,   0x0B);

    /* 6. Accel config — Bank 2
     *    SMPLRT_DIV=10 → ODR ≈ 102 Hz
     *    CONFIG: DLPF=001(111Hz), FS=01(±4g), FCHOICE=1 → 0x0B */
    wr(hi2c, B2_ACCEL_SMPLRT_2, 10);
    wr(hi2c, B2_ACCEL_CONFIG,   0x0B);

    /* 7. Enable I2C master (Bank 3 setup, then enable in USER_CTRL) */
    bank(hi2c, 0x30);
    wr(hi2c, B3_I2C_MST_CTRL, 0x07);  /* I2C master clk 345.6 kHz */
    bank(hi2c, 0x00);
    wr(hi2c, B0_USER_CTRL, 0x20);     /* I2C_MST_EN */
    HAL_Delay(10);

    /* 8. Reset + configure AK09916 at 100 Hz continuous */
    ak_write(hi2c, AK_CNTL2, 0x01);   /* soft reset */
    HAL_Delay(10);
    ak_write(hi2c, AK_CNTL2, AK_CNTL2_100HZ);
    HAL_Delay(10);

    /* 9. Slave 0 reads 9 bytes from AK ST1 every sample */
    bank(hi2c, 0x30);
    wr(hi2c, B3_SLV0_ADDR, AK_ADDR | 0x80);  /* read flag */
    wr(hi2c, B3_SLV0_REG,  AK_ST1);
    wr(hi2c, B3_SLV0_CTRL, 0x89);            /* enable, 9 bytes */
    bank(hi2c, 0x00);

    /* 10. INT pin: active-high, push-pull, latched, cleared on any read
     *     INT_ENABLE_1: RAW_DATA_0_RDY_EN */
    wr(hi2c, B0_INT_PIN_CFG,  0x12);
    wr(hi2c, B0_INT_ENABLE_1, 0x01);

    return HAL_OK;
}

/* ── Read ────────────────────────────────────────────────────────── */

HAL_StatusTypeDef ICM20948_Read(I2C_HandleTypeDef *hi2c,
                                ICM20948_Data     *d)
{
    /*
     * Burst read 23 bytes from ACCEL_XOUT_H (0x2D):
     *  [0..5]   accel X/Y/Z   (high,low × 3)
     *  [6..7]   temperature
     *  [8..13]  gyro  X/Y/Z
     *  [14..22] EXT_SLV0 shadow = AK ST1(1) + HX/HY/HZ(6) + ST2(1) + pad
     */
    uint8_t b[23];
    if (rd(hi2c, B0_ACCEL_XOUT_H, b, 23) != HAL_OK)
        return HAL_ERROR;

    /* Accel → m/s² */
    d->ax = (float)(int16_t)((b[0]  << 8) | b[1])  * ACCEL_SCALE_MS2;
    d->ay = (float)(int16_t)((b[2]  << 8) | b[3])  * ACCEL_SCALE_MS2;
    d->az = (float)(int16_t)((b[4]  << 8) | b[5])  * ACCEL_SCALE_MS2;

    /* Temperature */
    d->temp_c = (float)(int16_t)((b[6] << 8) | b[7])
                 * TEMP_SCALE + TEMP_OFFSET;

    /* Gyro → rad/s */
    d->gx = (float)(int16_t)((b[8]  << 8) | b[9])  * GYRO_SCALE_RADS;
    d->gy = (float)(int16_t)((b[10] << 8) | b[11]) * GYRO_SCALE_RADS;
    d->gz = (float)(int16_t)((b[12] << 8) | b[13]) * GYRO_SCALE_RADS;

    /* Magnetometer (little-endian, only if DRDY=1) */
    if (b[14] & 0x01) {
        d->mx = (float)(int16_t)((b[16] << 8) | b[15]) * MAG_SCALE_UT;
        d->my = (float)(int16_t)((b[18] << 8) | b[17]) * MAG_SCALE_UT;
        d->mz = (float)(int16_t)((b[20] << 8) | b[19]) * MAG_SCALE_UT;
    }

    d->timestamp_ms = HAL_GetTick();
    d->data_ready   = true;
    return HAL_OK;
}

/* ── Calibrate ───────────────────────────────────────────────────── */

bool ICM20948_Calibrate(const ICM20948_Data *d,
                        ICM20948_Calib      *c,
                        uint16_t             n_samples)
{
    if (c->done) return true;

    c->gx_bias += d->gx;
    c->gy_bias += d->gy;
    c->gz_bias += d->gz;
    c->gravity += sqrtf(d->ax*d->ax + d->ay*d->ay + d->az*d->az);
    c->count++;

    if (c->count >= n_samples) {
        float n      = (float)n_samples;
        c->gx_bias  /= n;
        c->gy_bias  /= n;
        c->gz_bias  /= n;
        c->gravity  /= n;
        c->done      = true;
    }
    return c->done;
}

/* ── Complementary filter ────────────────────────────────────────── */

void ICM20948_Filter(ICM20948_Data        *d,
                     const ICM20948_Calib *c,
                     float                 dt_s,
                     float                 alpha)
{
    if (!c->done || dt_s <= 0.0f || dt_s > 0.5f) return;

    float gx = d->gx - c->gx_bias;
    float gy = d->gy - c->gy_bias;

    float roll_acc  = atan2f(d->ay, d->az);
    float pitch_acc = atan2f(-d->ax, sqrtf(d->ay*d->ay + d->az*d->az));

    d->roll_rad  = alpha * (d->roll_rad  + gx * dt_s) + (1.0f - alpha) * roll_acc;
    d->pitch_rad = alpha * (d->pitch_rad + gy * dt_s) + (1.0f - alpha) * pitch_acc;
}
