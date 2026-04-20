/**
 * lcd_display.h — STM32F746G-DISCO Full Graphical Dashboard
 *
 * Screen layout (480 × 272 px):
 * ┌──────────────────────────────────────────────────────────────┐
 * │  STM32 TERRAIN BOT    RISK:[=====    ]         [ STATUS ]   │
 * ├───────────────────────┬──────────────────────────────────────┤
 * │  IMU ORIENTATION      │  WHEEL SPEEDS                        │
 * │  Roll : +000.0°       │  FL [==========>   ] +00.0          │
 * │  ░░░░░bar░░░░░        │  FR [==========>   ] +00.0          │
 * │  Pitch: +000.0°       │  RL [==========>   ] +00.0          │
 * │  ░░░░░bar░░░░░        │  RR [==========>   ] +00.0          │
 * │  Yaw  : +000.0°       │                                      │
 * │  ░░░░░bar░░░░░        │                                      │
 * ├───────────────────────┴──────────────────────────────────────┤
 * │  PITCH SAFETY    [=============>                          ]  │
 * └──────────────────────────────────────────────────────────────┘
 *
 * Colour coding:
 *   Green  = safe / forward
 *   Amber  = caution (>15° or >30% risk)
 *   Red    = danger  (>25° or >70% risk) / reverse
 *   Blue   = pitch bar
 *   Orange = roll bar
 *   Purple = yaw bar
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "madgwick.h"
#include "encoder.h"

/**
 * @brief Initialise LCD and draw static UI frame.
 *        Call once before main loop starts.
 */
void LCD_Display_Init(void);

/**
 * @brief Refresh all live data on screen.
 *        Call at ~10 Hz from main loop.
 *
 * @param ahrs         Madgwick filter state (quaternion → Euler)
 * @param calib_done   0=calibrating, 1=running
 * @param risk_scaled  Safety risk 0.0 (safe) to 1.0 (E-stop)
 */
void LCD_Display_Update(const Madgwick_t *ahrs,
                        uint8_t calib_done,
                        float risk_scaled);

#endif /* LCD_DISPLAY_H */
