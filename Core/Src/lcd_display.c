#include "lcd_display.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include "stm32746g_discovery_lcd.h"

#define LCD_W 480
#define LCD_H 272

static void draw_text(int x,int y,const char *s,uint32_t col)
{
    BSP_LCD_SetTextColor(col);
    BSP_LCD_DisplayStringAt(x,y,(uint8_t*)s,LEFT_MODE);
}

void LCD_Display_Init(void)
{
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER,LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
    BSP_LCD_DisplayOn();

    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetFont(&Font16);

    draw_text(10,5,"TERRAIN BOT DASHBOARD",LCD_COLOR_CYAN);
}

void LCD_Display_Update(const Madgwick_t *ahrs,
                        uint8_t calib_done,
                        float risk,
                        float ax,float ay,float az,
                        uint8_t enc_ok,
                        uint8_t host_ok)
{
    char buf[64];

    BSP_LCD_SetFont(&Font12);

    /* Risk */
    snprintf(buf,sizeof(buf),"Risk: %.2f",risk);
    BSP_LCD_FillRect(0,30,200,20);
    draw_text(10,30,buf,LCD_COLOR_WHITE);

    /* Motor Speeds */
    float vfl=Encoder_VelMmps(ENC_FL);
    float vfr=Encoder_VelMmps(ENC_FR);
    float vrl=Encoder_VelMmps(ENC_RL);
    float vrr=Encoder_VelMmps(ENC_RR);

    snprintf(buf,sizeof(buf),"FL %.1f mm/s",vfl);
    BSP_LCD_FillRect(0,60,200,20);
    draw_text(10,60,buf,LCD_COLOR_GREEN);

    snprintf(buf,sizeof(buf),"FR %.1f mm/s",vfr);
    BSP_LCD_FillRect(0,80,200,20);
    draw_text(10,80,buf,LCD_COLOR_GREEN);

    snprintf(buf,sizeof(buf),"RL %.1f mm/s",vrl);
    BSP_LCD_FillRect(0,100,200,20);
    draw_text(10,100,buf,LCD_COLOR_GREEN);

    snprintf(buf,sizeof(buf),"RR %.1f mm/s",vrr);
    BSP_LCD_FillRect(0,120,200,20);
    draw_text(10,120,buf,LCD_COLOR_GREEN);

    /* Encoder ticks */
    snprintf(buf,sizeof(buf),"FL ticks %ld",Encoder_Ticks(ENC_FL));
    BSP_LCD_FillRect(240,60,200,20);
    draw_text(250,60,buf,LCD_COLOR_YELLOW);

    snprintf(buf,sizeof(buf),"FR ticks %ld",Encoder_Ticks(ENC_FR));
    BSP_LCD_FillRect(240,80,200,20);
    draw_text(250,80,buf,LCD_COLOR_YELLOW);

    snprintf(buf,sizeof(buf),"RL ticks %ld",Encoder_Ticks(ENC_RL));
    BSP_LCD_FillRect(240,100,200,20);
    draw_text(250,100,buf,LCD_COLOR_YELLOW);

    snprintf(buf,sizeof(buf),"RR ticks %ld",Encoder_Ticks(ENC_RR));
    BSP_LCD_FillRect(240,120,200,20);
    draw_text(250,120,buf,LCD_COLOR_YELLOW);

    /* PWM percent */
    float pfl=(float)Motor_GetDuty(MOTOR_FL)/MOTOR_MAX_DUTY_APB2*100.0f;
    float pfr=(float)Motor_GetDuty(MOTOR_FR)/MOTOR_MAX_DUTY_APB1*100.0f;
    float prl=(float)Motor_GetDuty(MOTOR_RL)/MOTOR_MAX_DUTY_APB1*100.0f;
    float prr=(float)Motor_GetDuty(MOTOR_RR)/MOTOR_MAX_DUTY_APB1*100.0f;

    snprintf(buf,sizeof(buf),"PWM FL %.0f%%",pfl);
    BSP_LCD_FillRect(0,160,200,20);
    draw_text(10,160,buf,LCD_COLOR_CYAN);

    snprintf(buf,sizeof(buf),"PWM FR %.0f%%",pfr);
    BSP_LCD_FillRect(0,180,200,20);
    draw_text(10,180,buf,LCD_COLOR_CYAN);

    snprintf(buf,sizeof(buf),"PWM RL %.0f%%",prl);
    BSP_LCD_FillRect(0,200,200,20);
    draw_text(10,200,buf,LCD_COLOR_CYAN);

    snprintf(buf,sizeof(buf),"PWM RR %.0f%%",prr);
    BSP_LCD_FillRect(0,220,200,20);
    draw_text(10,220,buf,LCD_COLOR_CYAN);

    /* Status */
    snprintf(buf,sizeof(buf),"ENC:%s HOST:%s",
             enc_ok?"OK":"FAIL",
             host_ok?"OK":"TIMEOUT");

    BSP_LCD_FillRect(240,200,220,20);
    draw_text(250,200,buf,LCD_COLOR_WHITE);
}
