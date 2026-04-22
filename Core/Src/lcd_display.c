#include "lcd_display.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include "stm32746g_discovery_lcd.h"

#define LCD_W 480
#define LCD_H 272
#define NAV_Y 224
#define BACK_X 8
#define BACK_Y 230
#define BACK_W 126
#define BACK_H 34
#define IMU_X 356
#define IMU_Y 82
#define IMU_W 104
#define IMU_H 42
#define MOTOR_BTN_X 356
#define MOTOR_BTN_Y 134
#define MOTOR_BTN_W 104
#define MOTOR_BTN_H 42

typedef enum {
    LCD_PAGE_HOME = 0,
    LCD_PAGE_IMU = 1,
    LCD_PAGE_MOTORS = 2
} LcdPage_t;

static LcdPage_t g_page = LCD_PAGE_HOME;
static LcdPage_t g_drawn_page = (LcdPage_t)0xff;

static void draw_text(int x,int y,const char *s,uint32_t col)
{
    BSP_LCD_SetTextColor(col);
    BSP_LCD_DisplayStringAt(x,y,(uint8_t*)s,LEFT_MODE);
}

static void clear_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(x, y, w, h);
}

static void draw_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *label)
{
    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawRect(x, y, w, h);
    BSP_LCD_SetFont(&Font16);
    draw_text((int)(x + 14U), (int)(y + 12U), label, LCD_COLOR_WHITE);
}

static uint32_t risk_color(float risk, uint8_t emergency_stop)
{
    if (emergency_stop || risk >= 0.75f) return LCD_COLOR_RED;
    if (risk >= 0.35f) return LCD_COLOR_YELLOW;
    return LCD_COLOR_GREEN;
}

static const char *risk_label(float risk, uint8_t emergency_stop)
{
    if (emergency_stop || risk >= 0.75f) return emergency_stop ? "EMERGENCY STOP" : "STOP";
    if (risk >= 0.35f) return "MODERATE";
    return "SAFE";
}

static uint8_t point_in_rect(uint16_t px, uint16_t py,
                             uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    return (px >= x) && (px < (uint16_t)(x + w)) &&
           (py >= y) && (py < (uint16_t)(y + h));
}

void LCD_Display_Touch(uint16_t x, uint16_t y)
{
    if ((g_page == LCD_PAGE_HOME) && point_in_rect(x, y, IMU_X, IMU_Y, IMU_W, IMU_H)) {
        g_page = LCD_PAGE_IMU;
        g_drawn_page = (LcdPage_t)0xff;
    } else if ((g_page == LCD_PAGE_HOME) && point_in_rect(x, y, MOTOR_BTN_X, MOTOR_BTN_Y, MOTOR_BTN_W, MOTOR_BTN_H)) {
        g_page = LCD_PAGE_MOTORS;
        g_drawn_page = (LcdPage_t)0xff;
    } else if ((g_page != LCD_PAGE_HOME) && point_in_rect(x, y, BACK_X, BACK_Y, BACK_W, BACK_H)) {
        g_page = LCD_PAGE_HOME;
        g_drawn_page = (LcdPage_t)0xff;
    }
}

static void draw_static(void)
{
    if (g_drawn_page == g_page) return;

    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);
    draw_text(8, 8, "TERRAIN BOT", LCD_COLOR_WHITE);

    if (g_page == LCD_PAGE_HOME) {
        draw_text(8, 42, "DASHBOARD", LCD_COLOR_WHITE);
        BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
        BSP_LCD_DrawRect(8, 74, 326, 140);
        BSP_LCD_SetFont(&Font12);
        draw_text(16, 82, "Connections", LCD_COLOR_LIGHTGRAY);
        draw_button(IMU_X, IMU_Y, IMU_W, IMU_H, "IMU");
        draw_button(MOTOR_BTN_X, MOTOR_BTN_Y, MOTOR_BTN_W, MOTOR_BTN_H, "MOTOR");
        BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
        BSP_LCD_DrawRect(352, 186, 116, 28);
        BSP_LCD_SetFont(&Font12);
        draw_text(360, 194, "Status", LCD_COLOR_LIGHTGRAY);
    } else if (g_page == LCD_PAGE_IMU) {
        draw_text(8, 42, "IMU DATA", LCD_COLOR_WHITE);
        draw_button(BACK_X, BACK_Y, BACK_W, BACK_H, "< BACK");
    } else {
        draw_text(8, 42, "MOTOR DATA", LCD_COLOR_WHITE);
        draw_button(BACK_X, BACK_Y, BACK_W, BACK_H, "< BACK");
    }

    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
    BSP_LCD_DrawHLine(0, NAV_Y, LCD_W);
    g_drawn_page = g_page;
}

void LCD_Display_Init(void)
{
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER,LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
    BSP_LCD_DisplayOn();

    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);

    g_page = LCD_PAGE_HOME;
    g_drawn_page = (LcdPage_t)0xff;
    draw_static();
}

void LCD_Display_Update(const Madgwick_t *ahrs,
                        const ICM20948_Data *imu,
                        uint8_t calib_done,
                        float risk,
                        uint8_t emergency_stop,
                        uint8_t imu_ok,
                        uint8_t enc_ok,
                        uint8_t ros_ok,
                        uint8_t host_ok)
{
    char buf[64];
    uint32_t col = risk_color(risk, emergency_stop);
    float roll = ahrs->roll * (180.0f / 3.14159265f);
    float pitch = ahrs->pitch * (180.0f / 3.14159265f);
    float yaw = ahrs->yaw * (180.0f / 3.14159265f);
    float amag = sqrtf(imu->ax*imu->ax + imu->ay*imu->ay + imu->az*imu->az);

    draw_static();

    BSP_LCD_SetTextColor(col);
    BSP_LCD_FillRect(238, 6, 230, 24);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(col);
    draw_text(248, 10, risk_label(risk, emergency_stop), LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    BSP_LCD_SetFont(&Font12);

    if (g_page == LCD_PAGE_HOME) {
        clear_area(18, 94, 306, 114);
        clear_area(358, 202, 102, 10);
        clear_area(300, 42, 168, 24);
        BSP_LCD_SetTextColor(col);
        BSP_LCD_FillRect(300, 42, 168, 22);
        BSP_LCD_SetFont(&Font12);
        BSP_LCD_SetBackColor(col);
        draw_text(310, 48, risk_label(risk, emergency_stop), LCD_COLOR_BLACK);
        BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

        snprintf(buf,sizeof(buf),"IMU   : %s", imu_ok ? "CONNECTED" : "NOT CONNECTED");
        draw_text(18, 96, buf, imu_ok ? LCD_COLOR_GREEN : LCD_COLOR_RED);
        snprintf(buf,sizeof(buf),"ROS   : %s", ros_ok ? "CONNECTED" : "WAITING");
        draw_text(18, 118, buf, ros_ok ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW);
        snprintf(buf,sizeof(buf),"ESP32 : %s", enc_ok ? "CONNECTED" : "NOT CONNECTED");
        draw_text(18, 140, buf, enc_ok ? LCD_COLOR_GREEN : LCD_COLOR_RED);
        snprintf(buf,sizeof(buf),"TERRAIN: %s", risk_label(risk, emergency_stop));
        draw_text(18, 162, buf, col);
        snprintf(buf,sizeof(buf),"Risk %.2f", risk);
        draw_text(18, 184, buf, col);
        BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
        BSP_LCD_DrawRect(92, 184, 218, 14);
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_FillRect(93, 185, 216, 12);
        BSP_LCD_SetTextColor(col);
        BSP_LCD_FillRect(93, 185, (uint16_t)(216.0f * risk), 12);
        draw_text(358, 202, host_ok ? "CMD OK" : "NO CMD",
                  host_ok ? LCD_COLOR_GREEN : LCD_COLOR_YELLOW);
        return;
    }

    if (g_page == LCD_PAGE_IMU) {
        clear_area(8, 72, 462, 142);
        snprintf(buf,sizeof(buf),"Roll %7.2f  Pitch %7.2f  Yaw %7.2f", roll, pitch, yaw);
        draw_text(10, 76, buf, LCD_COLOR_WHITE);
        snprintf(buf,sizeof(buf),"ACC  X%+6.2f Y%+6.2f Z%+6.2f |A|%5.2f", imu->ax, imu->ay, imu->az, amag);
        draw_text(10, 104, buf, LCD_COLOR_GREEN);
        snprintf(buf,sizeof(buf),"GYR  X%+6.2f Y%+6.2f Z%+6.2f rad/s", imu->gx, imu->gy, imu->gz);
        draw_text(10, 128, buf, LCD_COLOR_CYAN);
        snprintf(buf,sizeof(buf),"MAG  X%+6.1f Y%+6.1f Z%+6.1f uT", imu->mx, imu->my, imu->mz);
        draw_text(10, 152, buf, LCD_COLOR_YELLOW);
        snprintf(buf,sizeof(buf),"Temp %.1f C  AHRS:%s  HOST:%s",
                 imu->temp_c, calib_done ? "RUN" : "CAL", host_ok ? "OK" : "TIMEOUT");
        draw_text(10, 180, buf, LCD_COLOR_WHITE);
        return;
    }

    /* Motor Speeds */
    float vfl=Encoder_VelMmps(ENC_FL);
    float vfr=Encoder_VelMmps(ENC_FR);
    float vrl=Encoder_VelMmps(ENC_RL);
    float vrr=Encoder_VelMmps(ENC_RR);

    clear_area(0, 60, 470, 160);
    snprintf(buf,sizeof(buf),"FL %.1f mm/s",vfl);
    draw_text(10,60,buf,LCD_COLOR_GREEN);
    snprintf(buf,sizeof(buf),"FR %.1f mm/s",vfr);
    draw_text(10,80,buf,LCD_COLOR_GREEN);
    snprintf(buf,sizeof(buf),"RL %.1f mm/s",vrl);
    draw_text(10,100,buf,LCD_COLOR_GREEN);
    snprintf(buf,sizeof(buf),"RR %.1f mm/s",vrr);
    draw_text(10,120,buf,LCD_COLOR_GREEN);

    /* Encoder ticks */
    snprintf(buf,sizeof(buf),"FL ticks %ld",Encoder_Ticks(ENC_FL));
    draw_text(250,60,buf,LCD_COLOR_YELLOW);

    snprintf(buf,sizeof(buf),"FR ticks %ld",Encoder_Ticks(ENC_FR));
    draw_text(250,80,buf,LCD_COLOR_YELLOW);

    snprintf(buf,sizeof(buf),"RL ticks %ld",Encoder_Ticks(ENC_RL));
    draw_text(250,100,buf,LCD_COLOR_YELLOW);

    snprintf(buf,sizeof(buf),"RR ticks %ld",Encoder_Ticks(ENC_RR));
    draw_text(250,120,buf,LCD_COLOR_YELLOW);

    /* PWM percent */
    float pfl=(float)Motor_GetDuty(MOTOR_FL)/MOTOR_MAX_DUTY_APB2*100.0f;
    float pfr=(float)Motor_GetDuty(MOTOR_FR)/MOTOR_MAX_DUTY_APB1*100.0f;
    float prl=(float)Motor_GetDuty(MOTOR_RL)/MOTOR_MAX_DUTY_APB1*100.0f;
    float prr=(float)Motor_GetDuty(MOTOR_RR)/MOTOR_MAX_DUTY_APB1*100.0f;

    snprintf(buf,sizeof(buf),"PWM FL %.0f%%",pfl);
    draw_text(10,160,buf,LCD_COLOR_CYAN);

    snprintf(buf,sizeof(buf),"PWM FR %.0f%%",pfr);
    draw_text(10,180,buf,LCD_COLOR_CYAN);

    snprintf(buf,sizeof(buf),"PWM RL %.0f%%",prl);
    draw_text(10,200,buf,LCD_COLOR_CYAN);

    snprintf(buf,sizeof(buf),"PWM RR %.0f%%",prr);
    draw_text(10,220,buf,LCD_COLOR_CYAN);

    /* Status */
    snprintf(buf,sizeof(buf),"ENC:%s HOST:%s",
             enc_ok?"OK":"FAIL",
             host_ok?"OK":"TIMEOUT");

    draw_text(250,200,buf,LCD_COLOR_WHITE);
}
