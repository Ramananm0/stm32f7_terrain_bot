/**
 * lcd_display.c — STM32F746G-DISCO Interactive Dashboard
 *
 * Layout (480 × 272 px):
 * ┌────────────────────────────────────────────────────────────┐
 * │  TERRAIN BOT  v2.0    RISK [████░░]  BAT ~12.4V  [STATUS] │ hdr
 * ├───────────────────────┬────────────────────────────────────┤
 * │  IMU ORIENTATION      │  MOTOR SPEEDS                      │
 * │  Roll  +00.0°  ░bar░  │  FL ►[████████░░░] +00.0 mm/s     │
 * │  Pitch +00.0°  ░bar░  │  FR ►[████████░░░] +00.0 mm/s     │
 * │  Yaw   +00.0°  ░bar░  │  RL ►[████████░░░] +00.0 mm/s     │
 * │                       │  RR ►[████████░░░] +00.0 mm/s     │
 * │  ACCEL (m/s²)         │  PWM DUTY                          │
 * │  aX +0.00 aY +0.00    │  FL [░░░░░░░░░░░] FR [░░░░░░░░░]  │
 * │  aZ +0.00             │  RL [░░░░░░░░░░░] RR [░░░░░░░░░]  │
 * ├───────────────────────┴────────────────────────────────────┤
 * │  SAFETY ◄pitch► [████████░░░░░░░░░░░░░░] -05.7° / 30°     │
 * └────────────────────────────────────────────────────────────┘
 */

#include "lcd_display.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "stm32746g_discovery_lcd.h"

/* ── Colours ─────────────────────────────────────────────────────── */
#define COL_BG          LCD_COLOR_BLACK
#define COL_HDR_BG      0xFF001533u
#define COL_HDR_TXT     0xFF00D4FFu
#define COL_SECTION     0xFF00D4FFu
#define COL_LABEL       0xFF888888u
#define COL_VALUE       LCD_COLOR_WHITE
#define COL_OK          0xFF00E676u
#define COL_WARN        0xFFFFAB00u
#define COL_DANGER      0xFFFF1744u
#define COL_CALIB       0xFFFFAB00u
#define COL_DIV         0xFF0A2A4Au

/* Bar colours */
#define COL_BAR_BG      0xFF0D1B2Au
#define COL_BAR_FWD     0xFF00E676u
#define COL_BAR_REV     0xFFFF1744u
#define COL_BAR_PITCH   0xFF448AFFu
#define COL_BAR_ROLL    0xFFFF6D00u
#define COL_BAR_YAW     0xFFE040FBu
#define COL_BAR_DUTY    0xFF00B0FFu
#define COL_BAR_RISK_LO 0xFF00E676u
#define COL_BAR_RISK_HI 0xFFFF1744u

/* ── Layout ──────────────────────────────────────────────────────── */
#define LCD_W    480
#define LCD_H    272
#define HDR_H    24
#define DIV_X    210    /* left/right panel divider */
#define BOT_Y    244    /* bottom safety bar area   */
#define MX       6      /* margin x                 */
#define BH       9      /* bar height               */
#define BM       2      /* bar margin               */

/* Left panel rows */
#define L_SEC_Y  (HDR_H + 4)
#define L_R_Y    (L_SEC_Y + 16)
#define L_P_Y    (L_R_Y + 14 + BH + 4)
#define L_Y_Y    (L_P_Y + 14 + BH + 4)
#define L_A_Y    (L_Y_Y + 14 + BH + 8)
#define L_AV_Y   (L_A_Y + 14)

/* Right panel rows */
#define R_X      (DIV_X + 6)
#define R_SEC_Y  (HDR_H + 4)
#define R_FL_Y   (R_SEC_Y + 16)
#define R_FR_Y   (R_FL_Y + 14 + BH + 4)
#define R_RL_Y   (R_FR_Y + 14 + BH + 4)
#define R_RR_Y   (R_RL_Y + 14 + BH + 4)
#define R_PWM_Y  (R_RR_Y + 14 + BH + 6)

/* Motor arrow symbols */
#define SYM_FWD   "\x10"   /* ► */
#define SYM_REV   "\x11"   /* ◄ */
#define SYM_STOP  "\xFE"   /* ■ */

/* ── Draw helpers ────────────────────────────────────────────────── */

static void bar_bidir(int x, int y, int w, float val, float max,
                      uint32_t cp, uint32_t cn)
{
    BSP_LCD_SetTextColor(COL_BAR_BG);
    BSP_LCD_FillRect(x, y, w, BH);
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DrawVLine(x + w/2, y, BH);
    if (max == 0.0f) return;
    if (val >  max) val =  max;
    if (val < -max) val = -max;
    int half = w/2;
    int fill = (int)(val / max * (float)half);
    if (fill > 0) {
        BSP_LCD_SetTextColor(cp);
        BSP_LCD_FillRect(x + half, y + BM, fill, BH - BM*2);
    } else if (fill < 0) {
        BSP_LCD_SetTextColor(cn);
        BSP_LCD_FillRect(x + half + fill, y + BM, -fill, BH - BM*2);
    }
}

static void bar_unidir(int x, int y, int w, float val, float max,
                       uint32_t col)
{
    BSP_LCD_SetTextColor(COL_BAR_BG);
    BSP_LCD_FillRect(x, y, w, BH);
    if (max == 0.0f) return;
    float r = fabsf(val) / max;
    if (r > 1.0f) r = 1.0f;
    int fill = (int)(r * (float)w);
    if (fill > 0) {
        BSP_LCD_SetTextColor(col);
        BSP_LCD_FillRect(x, y + BM, fill, BH - BM*2);
    }
}

static uint32_t angle_col(float deg)
{
    float a = fabsf(deg);
    if (a >= 25.0f) return COL_DANGER;
    if (a >= 15.0f) return COL_WARN;
    return COL_OK;
}

static void draw_imu_row(int y, const char *lbl, float deg,
                         float max, uint32_t bar_col)
{
    char buf[20];
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(MX, y, DIV_X - MX*2, 14);
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(MX, y, (uint8_t*)lbl, LEFT_MODE);
    snprintf(buf, sizeof(buf), "%+6.1f\xB0", (double)deg);
    BSP_LCD_SetTextColor(angle_col(deg));
    BSP_LCD_DisplayStringAt(MX + 56, y, (uint8_t*)buf, LEFT_MODE);
    bar_bidir(MX, y + 14, DIV_X - MX*2, deg, max, bar_col, bar_col);
}

static void draw_motor_row(int y, const char *lbl, float vel,
                           float max_v)
{
    char buf[20];
    int bar_x   = R_X + 22;
    int bar_w   = 130;
    int val_x   = bar_x + bar_w + 3;

    /* Arrow direction indicator */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(R_X, y, LCD_W - R_X - MX, 14);
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(R_X, y, (uint8_t*)lbl, LEFT_MODE);

    /* Direction symbol */
    const char *sym = (vel > 0.5f)  ? SYM_FWD  :
                      (vel < -0.5f) ? SYM_REV  : SYM_STOP;
    uint32_t sc = (vel > 0.5f)  ? COL_BAR_FWD :
                  (vel < -0.5f) ? COL_BAR_REV : COL_LABEL;
    BSP_LCD_SetTextColor(sc);
    BSP_LCD_DisplayStringAt(R_X + 16, y, (uint8_t*)sym, LEFT_MODE);

    /* Speed bar */
    uint32_t bc = (vel >= 0.0f) ? COL_BAR_FWD : COL_BAR_REV;
    bar_bidir(bar_x, y + 14, bar_w, vel, max_v, COL_BAR_FWD, COL_BAR_REV);

    /* Value */
    snprintf(buf, sizeof(buf), "%+5.1f", (double)vel);
    BSP_LCD_SetTextColor(COL_VALUE);
    BSP_LCD_DisplayStringAt(val_x, y, (uint8_t*)buf, LEFT_MODE);
    (void)bc;
}

static void draw_pwm_mini(int x, int y, int w, const char *lbl,
                          float vel, float max_v)
{
    char buf[10];
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(x, y, (uint8_t*)lbl, LEFT_MODE);
    int bx = x + 18;
    int bw = w - 18;
    float duty_pct = fabsf(vel) / max_v * 85.7f; /* 85.7% max */
    bar_unidir(bx, y + 2, bw, duty_pct, 100.0f, COL_BAR_DUTY);
    snprintf(buf, sizeof(buf), "%3.0f%%", (double)duty_pct);
    BSP_LCD_SetTextColor(COL_BAR_DUTY);
    BSP_LCD_DisplayStringAt(bx + bw + 2, y, (uint8_t*)buf, LEFT_MODE);
}

/* ── Init ────────────────────────────────────────────────────────── */
void LCD_Display_Init(void)
{
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
    BSP_LCD_DisplayOn();

    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(0, 0, LCD_W, LCD_H);

    /* Header */
    BSP_LCD_SetTextColor(COL_HDR_BG);
    BSP_LCD_FillRect(0, 0, LCD_W, HDR_H);
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetTextColor(COL_HDR_TXT);
    BSP_LCD_SetBackColor(COL_HDR_BG);
    BSP_LCD_DisplayStringAt(MX, 6, (uint8_t*)"TERRAIN BOT v2.0", LEFT_MODE);

    /* Static labels */
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetBackColor(COL_BG);

    /* Divider */
    BSP_LCD_SetTextColor(COL_DIV);
    BSP_LCD_DrawVLine(DIV_X, HDR_H, BOT_Y - HDR_H);
    BSP_LCD_DrawHLine(0, BOT_Y - 2, LCD_W);

    /* Section titles */
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_DisplayStringAt(MX, L_SEC_Y, (uint8_t*)"IMU ORIENTATION", LEFT_MODE);
    BSP_LCD_DisplayStringAt(R_X, R_SEC_Y, (uint8_t*)"MOTOR SPEEDS", LEFT_MODE);

    /* Bottom label */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DisplayStringAt(MX, BOT_Y + 1,
        (uint8_t*)"PITCH SAFETY  E-STOP >30", LEFT_MODE);
}

/* ── Update ──────────────────────────────────────────────────────── */
void LCD_Display_Update(const Madgwick_t *ahrs,
                        uint8_t calib_done,
                        float risk,
                        float ax, float ay, float az)
{
    char buf[32];

    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetBackColor(COL_BG);

    /* ── Header status ── */
    BSP_LCD_SetTextColor(COL_HDR_BG);
    BSP_LCD_FillRect(LCD_W - 82, 0, 82, HDR_H);
    BSP_LCD_SetBackColor(COL_HDR_BG);
    if (calib_done) {
        BSP_LCD_SetTextColor(COL_OK);
        BSP_LCD_DisplayStringAt(LCD_W - 72, 6, (uint8_t*)"[ RUN ]", LEFT_MODE);
    } else {
        BSP_LCD_SetTextColor(COL_CALIB);
        BSP_LCD_DisplayStringAt(LCD_W - 72, 6, (uint8_t*)"[CALIB]", LEFT_MODE);
    }

    /* Risk bar in header */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_SetBackColor(COL_HDR_BG);
    BSP_LCD_DisplayStringAt(170, 6, (uint8_t*)"RISK", LEFT_MODE);
    uint32_t rc = risk > 0.7f ? COL_DANGER :
                  risk > 0.3f ? COL_WARN   : COL_BAR_RISK_LO;
    bar_unidir(198, 7, 60, risk, 1.0f, rc);

    /* Battery indicator */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DisplayStringAt(265, 6, (uint8_t*)"BAT ~12.4V", LEFT_MODE);

    BSP_LCD_SetBackColor(COL_BG);

    /* ── Euler angles ── */
    float q0=ahrs->q0, q1=ahrs->q1, q2=ahrs->q2, q3=ahrs->q3;
    float roll  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))
                  * (180.0f/3.14159265f);
    float pitch = asinf(2.0f*(q0*q2-q3*q1))
                  * (180.0f/3.14159265f);
    float yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))
                  * (180.0f/3.14159265f);

    draw_imu_row(L_R_Y, "Roll :", roll,  45.0f, COL_BAR_ROLL);
    draw_imu_row(L_P_Y, "Pitch:", pitch, 45.0f, COL_BAR_PITCH);
    draw_imu_row(L_Y_Y, "Yaw  :", yaw,  180.0f, COL_BAR_YAW);

    /* ── Accel values ── */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(MX, L_A_Y, DIV_X - MX*2, 28);
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_DisplayStringAt(MX, L_A_Y, (uint8_t*)"ACCEL (m/s²)", LEFT_MODE);
    BSP_LCD_SetTextColor(COL_VALUE);
    snprintf(buf, sizeof(buf), "X%+5.2f Y%+5.2f", (double)ax, (double)ay);
    BSP_LCD_DisplayStringAt(MX, L_AV_Y, (uint8_t*)buf, LEFT_MODE);
    snprintf(buf, sizeof(buf), "Z%+5.2f", (double)az);
    BSP_LCD_DisplayStringAt(MX, L_AV_Y + 13, (uint8_t*)buf, LEFT_MODE);

    /* ── Motor speed rows ── */
    float max_v = MOTOR_MAX_SPEED_MMPS;
    draw_motor_row(R_FL_Y, "FL", Encoder_VelMmps(ENC_FL), max_v);
    draw_motor_row(R_FR_Y, "FR", Encoder_VelMmps(ENC_FR), max_v);
    draw_motor_row(R_RL_Y, "RL", Encoder_VelMmps(ENC_RL), max_v);
    draw_motor_row(R_RR_Y, "RR", Encoder_VelMmps(ENC_RR), max_v);

    /* ── PWM duty mini bars ── */
    int hw = (LCD_W - R_X - MX*2 - 4) / 2;
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(R_X, R_PWM_Y, LCD_W - R_X - MX, 28);
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_DisplayStringAt(R_X, R_PWM_Y,
        (uint8_t*)"PWM DUTY (max 85.7%)", LEFT_MODE);
    draw_pwm_mini(R_X,      R_PWM_Y + 13, hw, "FL", Encoder_VelMmps(ENC_FL), max_v);
    draw_pwm_mini(R_X + hw + 4, R_PWM_Y + 13, hw, "FR", Encoder_VelMmps(ENC_FR), max_v);
    /* second row RL RR would go at +26 but space is tight — skip for now */

    /* ── Bottom safety pitch bar ── */
    uint32_t pc = fabsf(pitch) > 25.0f ? COL_DANGER :
                  fabsf(pitch) > 15.0f ? COL_WARN   : COL_OK;
    bar_bidir(160, BOT_Y + 3, 270, pitch, 30.0f, pc, pc);
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(LCD_W - 64, BOT_Y, 64, 20);
    BSP_LCD_SetTextColor(pc);
    snprintf(buf, sizeof(buf), "%+5.1f\xB0", (double)pitch);
    BSP_LCD_DisplayStringAt(LCD_W - 60, BOT_Y + 3, (uint8_t*)buf, LEFT_MODE);
}
