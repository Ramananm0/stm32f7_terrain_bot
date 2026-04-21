# STM32F7 Terrain Bot — Verified Pin Connections

This file documents the **actual hardware wiring** expected by firmware.

---

# Motor PWM Connections (STM32 → BTS7960)

## Front Left (FL)

| STM32 Pin | Function | BTS Pin |
|-----------|----------|----------|
| PF6 | TIM10_CH1 | RPWM |
| PF7 | TIM11_CH1 | LPWM |
| 3.3V | Enable | R_EN |
| 3.3V | Enable | L_EN |
| GND | Ground | GND |

---

## Front Right (FR)

| STM32 Pin | Function | BTS Pin |
|-----------|----------|----------|
| PF8 | TIM13_CH1 | RPWM |
| PF9 | TIM14_CH1 | LPWM |
| 3.3V | Enable | R_EN |
| 3.3V | Enable | L_EN |
| GND | Ground | GND |

---

## Rear Left (RL)

| STM32 Pin | Function | BTS Pin |
|-----------|----------|----------|
| PH6 | TIM12_CH2 | RPWM |
| PB15 | TIM12_CH1 | LPWM |
| 3.3V | Enable | R_EN |
| 3.3V | Enable | L_EN |
| GND | Ground | GND |

---

## Rear Right (RR)

| STM32 Pin | Function | BTS Pin |
|-----------|----------|----------|
| PA15 | TIM2_CH1 | RPWM |
| PB4 | TIM3_CH1 | LPWM |
| 3.3V | Enable | R_EN |
| 3.3V | Enable | L_EN |
| GND | Ground | GND |

---

# UART Connections (ESP32 ↔ STM32)

## Encoder UART

| ESP32 | STM32 |
|-------|--------|
| TX | PB7 (RX) |
| RX | PB6 (TX) |
| GND | GND |

Baud Rate:

115200

---

## Host Command UART

| ESP32 | STM32 |
|-------|--------|
| TX | PD6 (RX) |
| RX | PD5 (TX) |
| GND | GND |

---

# Power Connections

Battery → Switch → Power Distribution Board

From PDB:

- BTS7960 #1
- BTS7960 #2
- BTS7960 #3
- BTS7960 #4

Buck Converter Outputs:

- 5V → Raspberry Pi
- 5V → ESP32

---

# Notes

- All grounds must be common.
- Use thick wires (≥1 mm²) for motor power.
- Logic wiring can use 24–26 AWG.

---

# Status

This mapping matches firmware commit:

motor.c
lcd_display.c
app.c

If hardware differs, update this file accordingly.
