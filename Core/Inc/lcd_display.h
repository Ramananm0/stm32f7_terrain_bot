/**
 * lcd_display.h — Full graphical terrain bot dashboard
 * STM32F746G-DISCO 480×272 built-in LCD
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "madgwick.h"
#include "encoder.h"
#include "motor.h"

void LCD_Display_Init(void);

/**
 * @param ahrs        Madgwick filter state
 * @param calib_done  0=calibrating 1=running
 * @param risk        Safety risk 0.0-1.0
 * @param ax,ay,az    Accelerometer m/s² (raw from IMU)
 */
void LCD_Display_Update(const Madgwick_t *ahrs,
                        uint8_t calib_done,
                        float risk,
                        float ax, float ay, float az);

#endif /* LCD_DISPLAY_H */
