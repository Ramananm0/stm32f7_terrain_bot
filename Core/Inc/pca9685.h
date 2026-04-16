/**
 * pca9685.h — PCA9685 16-channel I2C PWM controller driver
 * STM32F746G-DISCO  |  I2C1 (PB8=SCL, PB9=SDA), address 0x40
 *
 * ── Why we need this ────────────────────────────────────────────────────
 *   The STM32F746G-DISCO uses many Port-E and Port-I pins internally:
 *     PE11, PE13, PE14 → LTDC (LCD pixel clock / data enable / green data)
 *     PI2,  PI3        → FMC  (external SDRAM data bus)
 *   These overlap with TIM1_CH2–CH4 and the Arduino D7/D8 header pins,
 *   making it impossible to use those pins for motor PWM/DIR.
 *   The PCA9685 routes all motor signals over I2C instead.
 *
 * ── Hardware wiring ─────────────────────────────────────────────────────
 *   PCA9685 module → STM32F746G-DISCO
 *     VCC    → 3V3
 *     GND    → GND
 *     SCL    → PB8  (Arduino SCL header)
 *     SDA    → PB9  (Arduino SDA header)
 *     OE     → GND  (always enabled)
 *     A0-A5  → GND  (I2C address = 0x40)
 *
 * ── Channel assignment ──────────────────────────────────────────────────
 *   CH0 : Motor FL  PWM   (speed,     0–4095)
 *   CH1 : Motor FR  PWM
 *   CH2 : Motor RL  PWM
 *   CH3 : Motor RR  PWM
 *   CH4 : Motor FL  DIR   (0 = fwd,  4095 = rev)
 *   CH5 : Motor FR  DIR
 *   CH6 : Motor RL  DIR
 *   CH7 : Motor RR  DIR
 *
 * ── PWM frequency ───────────────────────────────────────────────────────
 *   Set to 1 kHz (PRESCALE = 5, internal 25 MHz osc):
 *     f = 25 000 000 / (4096 × (5+1)) ≈ 1017 Hz
 *   Adequate for RMCS-3070 brushed-DC motors (inductance + gearbox filter).
 */

#ifndef PCA9685_H
#define PCA9685_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ── I2C address (7-bit, shift left before HAL transmit) ─────────────── */
#define PCA9685_I2C_ADDR    0x40u

/* ── Register map ───────────────────────────────────────────────────────
 *  MODE1    0x00   MODE2    0x01
 *  LEDn_ON_L  = 0x06 + 4*n    LEDn_ON_H  = 0x07 + 4*n
 *  LEDn_OFF_L = 0x08 + 4*n    LEDn_OFF_H = 0x09 + 4*n
 *  PRESCALE   0xFE
 */
#define PCA9685_REG_MODE1    0x00u
#define PCA9685_REG_PRESCALE 0xFEu
#define PCA9685_LED0_ON_L    0x06u    /* base register for channel 0 */

/* MODE1 bits */
#define PCA9685_MODE1_SLEEP  0x10u
#define PCA9685_MODE1_AI     0x20u    /* auto-increment */
#define PCA9685_MODE1_RESTART 0x80u

/* Full-ON / Full-OFF markers (bit 12 of ON_H or OFF_H word) */
#define PCA9685_FULL_ON      0x1000u
#define PCA9685_FULL_OFF     0x1000u

/* ── Resolution ─────────────────────────────────────────────────────── */
#define PCA9685_MAX_DUTY     4095u   /* 12-bit */

/* ── API ────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise PCA9685 at 1 kHz PWM.
 *         Call after MX_I2C1_Init() but before Motor_Init().
 * @param  hi2c  I2C handle (I2C1, 400 kHz fast-mode)
 */
void PCA9685_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Set one PWM channel.
 * @param  channel  0..15
 * @param  on       Tick count where output goes HIGH (0..4095)
 * @param  off      Tick count where output goes LOW  (0..4095)
 *                  Typical usage: on=0, off=duty  (duty=0..4095)
 */
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief  Set a channel to a simple duty cycle (on=0, off=duty).
 * @param  channel  0..15
 * @param  duty     0 = off, 4095 = full on
 */
void PCA9685_SetDuty(uint8_t channel, uint16_t duty);

/**
 * @brief  Force a channel fully ON (logic HIGH) or fully OFF (logic LOW).
 *         Used for direction pins. faster than SetDuty(ch, 4095/0).
 * @param  channel  0..15
 * @param  state    0 = LOW, 1 = HIGH
 */
void PCA9685_SetDigital(uint8_t channel, uint8_t state);

#endif /* PCA9685_H */
