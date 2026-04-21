#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "madgwick.h"

void LCD_Display_Init(void);

void LCD_Display_Update(const Madgwick_t *ahrs,
                        uint8_t calib_done,
                        float risk,
                        float ax, float ay, float az,
                        uint8_t enc_ok,
                        uint8_t host_ok);

#endif /* LCD_DISPLAY_H */
