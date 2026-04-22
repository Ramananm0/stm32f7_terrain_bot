#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include "icm20948.h"
#include "madgwick.h"

void LCD_Display_Init(void);

void LCD_Display_Update(const Madgwick_t *ahrs,
                        const ICM20948_Data *imu,
                        uint8_t calib_done,
                        float risk,
                        uint8_t emergency_stop,
                        uint8_t imu_ok,
                        uint8_t enc_ok,
                        uint8_t ros_ok,
                        uint8_t host_ok);

#endif /* LCD_DISPLAY_H */
