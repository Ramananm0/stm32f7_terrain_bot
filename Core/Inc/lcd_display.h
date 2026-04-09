/**
 * lcd_display.h — STM32F746G-DISCO LCD Dashboard
 *
 * Displays live IMU (roll/pitch/yaw) and wheel velocity data
 * on the built-in 4.3" LCD (480x272, LTDC via BSP).
 *
 * Requires: stm32746g_discovery_lcd BSP in your CubeMX project.
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "madgwick.h"
#include "encoder.h"

/**
 * LCD_Display_Init — initialise LCD and draw static UI frame.
 * Call once before App_Run() enters its main loop.
 */
void LCD_Display_Init(void);

/**
 * LCD_Display_Update — refresh all live values on screen.
 * Call at ~10 Hz from the main loop (no need to go faster, LCD is 60 Hz).
 *
 * @param ahrs   Pointer to Madgwick filter (quaternion → Euler conversion done here)
 * @param calib_done  Pass true after gyro calibration finishes
 */
void LCD_Display_Update(const Madgwick_t *ahrs, uint8_t calib_done);

#endif /* LCD_DISPLAY_H */
