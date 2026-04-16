/**
 * pca9685.c — PCA9685 16-channel I2C PWM controller driver
 * STM32F746G-DISCO  |  I2C1 (PB8=SCL, PB9=SDA)
 */
#include "pca9685.h"

static I2C_HandleTypeDef *g_hi2c = NULL;

/* ── Internal helpers ─────────────────────────────────────────────────── */

static void write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    HAL_I2C_Master_Transmit(g_hi2c,
                            (uint16_t)(PCA9685_I2C_ADDR << 1),
                            buf, 2, 10);
}

/* ── Public API ───────────────────────────────────────────────────────── */

void PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
    g_hi2c = hi2c;

    /* Software reset via General Call address 0x00, byte 0x06 */
    uint8_t reset = 0x06u;
    HAL_I2C_Master_Transmit(g_hi2c, 0x00u, &reset, 1, 10);
    HAL_Delay(10);

    /* Sleep mode — required to change PRESCALE */
    write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP);

    /*
     * PRESCALE = round(25 000 000 / (4096 × f_pwm)) − 1
     * For f_pwm = 1000 Hz  →  PRESCALE = round(6.104) − 1 = 5
     * Actual: 25 000 000 / (4096 × 6) ≈ 1017 Hz
     */
    write_reg(PCA9685_REG_PRESCALE, 5u);

    /* Wake up — clear SLEEP, enable auto-increment */
    write_reg(PCA9685_REG_MODE1, PCA9685_MODE1_AI);
    HAL_Delay(1);   /* oscillator stabilise: ≥ 500 µs */

    /* All channels off at startup */
    for (uint8_t ch = 0; ch < 16u; ch++) {
        PCA9685_SetPWM(ch, 0u, 0u);
    }
}

void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (g_hi2c == NULL || channel > 15u) return;

    uint8_t buf[5];
    buf[0] = (uint8_t)(PCA9685_LED0_ON_L + 4u * channel);
    buf[1] = (uint8_t)( on        & 0xFFu);
    buf[2] = (uint8_t)((on  >> 8) & 0x1Fu);
    buf[3] = (uint8_t)( off       & 0xFFu);
    buf[4] = (uint8_t)((off >> 8) & 0x1Fu);

    HAL_I2C_Master_Transmit(g_hi2c,
                            (uint16_t)(PCA9685_I2C_ADDR << 1),
                            buf, 5u, 10);
}

void PCA9685_SetDuty(uint8_t channel, uint16_t duty)
{
    if (duty > PCA9685_MAX_DUTY) duty = PCA9685_MAX_DUTY;
    PCA9685_SetPWM(channel, 0u, duty);
}

void PCA9685_SetDigital(uint8_t channel, uint8_t state)
{
    if (state) {
        /* Full ON: LEDn_ON_H bit 12 = 1, LEDn_OFF = 0 */
        PCA9685_SetPWM(channel,
                       (uint16_t)PCA9685_FULL_ON,
                       0u);
    } else {
        /* Full OFF: LEDn_ON = 0, LEDn_OFF_H bit 12 = 1 */
        PCA9685_SetPWM(channel,
                       0u,
                       (uint16_t)PCA9685_FULL_OFF);
    }
}
