/**
 * icm20948.h  —  ICM-20948 9-axis IMU driver
 * Board  : STM32F746G-DISCO
 * Bus    : I2C1   SCL=PB8 (Arduino SCL pin)  SDA=PB9 (Arduino SDA pin)
 * INT pin: D2/PG6  (EXTI, optional)
 * AD0    : GND    → I2C address 0x68
 *
 * Sensor config:
 *   Accel  :  ±4 g        DLPF 111 Hz   ODR ~102 Hz
 *   Gyro   :  ±500 dps    DLPF 119 Hz   ODR ~102 Hz
 *   Mag    :  AK09916     continuous 100 Hz  (via I2C master)
 */
#ifndef ICM20948_H
#define ICM20948_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── I2C address ─────────────────────────────────────── */
#define ICM20948_ADDR       (0x68 << 1)   /* AD0=GND */

/* ── Register map ────────────────────────────────────── */
/* Bank select (any bank) */
#define REG_BANK_SEL        0x7F

/* Bank 0 */
#define B0_WHO_AM_I         0x00   /* expected: 0xEA */
#define B0_USER_CTRL        0x03
#define B0_PWR_MGMT_1       0x06
#define B0_PWR_MGMT_2       0x07
#define B0_INT_PIN_CFG      0x0F
#define B0_INT_ENABLE_1     0x11
#define B0_ACCEL_XOUT_H     0x2D   /* 14 bytes: accel+temp+gyro */
#define B0_EXT_SLV_DATA_00  0x3B   /* AK09916 shadow (9 bytes) */

/* Bank 2 */
#define B2_GYRO_SMPLRT_DIV  0x00
#define B2_GYRO_CONFIG_1    0x01
#define B2_ACCEL_SMPLRT_2   0x11
#define B2_ACCEL_CONFIG     0x14

/* Bank 3 (I2C master to AK09916) */
#define B3_I2C_MST_CTRL     0x01
#define B3_SLV0_ADDR        0x03
#define B3_SLV0_REG         0x04
#define B3_SLV0_CTRL        0x05
#define B3_SLV0_DO          0x06

/* AK09916 */
#define AK_ADDR             0x0C
#define AK_CNTL2            0x31
#define AK_CNTL2_100HZ      0x08
#define AK_ST1              0x10
#define AK_HXL              0x11

/* ── Scale factors ───────────────────────────────────── */
#define ACCEL_SCALE_MS2     (4.0f / 32768.0f * 9.80665f)   /* ±4g  → m/s² */
#define GYRO_SCALE_RADS     (500.0f / 32768.0f * 0.017453293f) /* ±500dps → rad/s */
#define MAG_SCALE_UT        (0.15f)                          /* µT/LSB */
#define TEMP_SCALE          (1.0f / 333.87f)
#define TEMP_OFFSET         21.0f

/* ── Data ────────────────────────────────────────────── */
typedef struct {
    /* SI units */
    float ax, ay, az;          /* m/s²  */
    float gx, gy, gz;          /* rad/s */
    float mx, my, mz;          /* µT    */
    float temp_c;

    /* Complementary filter output */
    float roll_rad;
    float pitch_rad;

    uint32_t timestamp_ms;
    bool     data_ready;
} ICM20948_Data;

/* Gyro bias + gravity calibration */
typedef struct {
    float  gx_bias, gy_bias, gz_bias;  /* rad/s */
    float  gravity;                     /* m/s²  */
    bool   done;
    uint16_t count;
} ICM20948_Calib;

/* ── API ─────────────────────────────────────────────── */

/* Initialise sensor — returns HAL_OK or HAL_ERROR */
HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef *hi2c);

/* Read accel + gyro + mag into data struct */
HAL_StatusTypeDef ICM20948_Read(I2C_HandleTypeDef *hi2c,
                                ICM20948_Data     *d);

/* Accumulate samples for gyro bias.  Call while robot is still.
   Returns true when n_samples collected and calibration is done. */
bool ICM20948_Calibrate(const ICM20948_Data *d,
                        ICM20948_Calib      *c,
                        uint16_t             n_samples);

/* Complementary filter — legacy, kept for reference.
   App_Run() uses Madgwick AHRS instead of this function.
   Call every cycle after ICM20948_Read() if you want roll/pitch
   without the full Madgwick filter. */
void ICM20948_Filter(ICM20948_Data        *d,
                     const ICM20948_Calib *c,
                     float                 dt_s,
                     float                 alpha);

#endif /* ICM20948_H */
