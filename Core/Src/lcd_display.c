/**
 * lcd_display.c — STM32F746G-DISCO LCD Dashboard
 *
 * Layout (480 × 272 px):
 * ┌─────────────────────────────────────────┐
 * │       STM32 TERRAIN BOT  [STATUS]       │  row 0  (header bar)
 * ├─────────────────────────────────────────┤
 * │  IMU — ORIENTATION                      │  row 1  (section title)
 * │   Roll  :  +000.00 deg                  │  row 2
 * │   Pitch :  +000.00 deg                  │  row 3
 * │   Yaw   :  +000.00 deg                  │  row 4
 * ├─────────────────────────────────────────┤
 * │  WHEEL VELOCITY                         │  row 5
 * │   Left  :  +000.00 mm/s                 │  row 6
 * │   Right :  +000.00 mm/s                 │  row 7
 * ├─────────────────────────────────────────┤
 * │  pitch bar  [=========>    ]            │  row 8  (visual bar)
 * └─────────────────────────────────────────┘
 */

#include "lcd_display.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>

/* BSP LCD driver — comes with STM32CubeF7 BSP for 746G-DISCO */
#include "stm32746g_discovery_lcd.h"

/* ── Colours ──────────────────────────────────────────────────────── */
#define COL_BG          LCD_COLOR_BLACK
#define COL_HEADER_BG   0xFF003366u    /* dark blue */
#define COL_HEADER_TXT  LCD_COLOR_WHITE
#define COL_SECTION     0xFF00AABB u   /* cyan-ish  */
#define COL_LABEL       0xFFCCCCCCu   /* light grey */
#define COL_VALUE       LCD_COLOR_WHITE
#define COL_OK          0xFF00CC44u    /* green  */
#define COL_CALIB       0xFFFFAA00u   /* amber  */
#define COL_BAR_BG      0xFF333333u
#define COL_BAR_FG      0xFF00CC44u
#define COL_NEG_BAR     0xFFFF4444u

/* ── Layout constants ─────────────────────────────────────────────── */
#define LCD_W           480
#define LCD_H           272

#define ROW_H           28             /* pixels per text row        */
#define MARGIN_X        12

/* Y positions of each row */
#define Y_HEADER        0
#define Y_IMU_TITLE     (ROW_H)
#define Y_ROLL          (ROW_H * 2)
#define Y_PITCH         (ROW_H * 3)
#define Y_YAW           (ROW_H * 4)
#define Y_WHL_TITLE     (ROW_H * 5 + 4)
#define Y_WHL_LEFT      (ROW_H * 6 + 4)
#define Y_WHL_RIGHT     (ROW_H * 7 + 4)
#define Y_BAR           (ROW_H * 8 + 8)
#define BAR_H           20
#define BAR_X           MARGIN_X
#define BAR_W           (LCD_W - MARGIN_X * 2)

/* ── Internal helpers ─────────────────────────────────────────────── */

/** Draw a filled horizontal bar representing a value in [-max, +max]. */
static void draw_hbar(float value, float max_val)
{
    /* background */
    BSP_LCD_SetTextColor(COL_BAR_BG);
    BSP_LCD_FillRect(BAR_X, Y_BAR, BAR_W, BAR_H);

    /* centre line */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DrawVLine(BAR_X + BAR_W / 2, Y_BAR, BAR_H);

    /* clamp */
    if (value >  max_val) value =  max_val;
    if (value < -max_val) value = -max_val;

    int centre = BAR_X + BAR_W / 2;
    int fill   = (int)((value / max_val) * (BAR_W / 2));

    BSP_LCD_SetTextColor(fill >= 0 ? COL_BAR_FG : COL_NEG_BAR);
    if (fill >= 0)
        BSP_LCD_FillRect(centre, Y_BAR + 2, fill, BAR_H - 4);
    else
        BSP_LCD_FillRect(centre + fill, Y_BAR + 2, -fill, BAR_H - 4);
}

/** Overwrite a value field without redrawing the whole screen. */
static void draw_value(int y, const char *label, const char *val_str)
{
    char buf[40];

    /* Erase value area only (right half of row) */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(MARGIN_X + 100, y, LCD_W - MARGIN_X - 100, ROW_H - 2);

    /* Label */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(MARGIN_X, y, (uint8_t *)label, LEFT_MODE);

    /* Value */
    BSP_LCD_SetTextColor(COL_VALUE);
    BSP_LCD_DisplayStringAt(MARGIN_X + 100, y, (uint8_t *)val_str, LEFT_MODE);
    (void)buf;
}

/* ── Public API ───────────────────────────────────────────────────── */

void LCD_Display_Init(void)
{
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
    BSP_LCD_DisplayOn();

    /* Fill background */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(0, 0, LCD_W, LCD_H);

    /* ── Header bar ── */
    BSP_LCD_SetTextColor(COL_HEADER_BG);
    BSP_LCD_FillRect(0, Y_HEADER, LCD_W, ROW_H);
    BSP_LCD_SetFont(&Font20);
    BSP_LCD_SetTextColor(COL_HEADER_TXT);
    BSP_LCD_SetBackColor(COL_HEADER_BG);
    BSP_LCD_DisplayStringAt(0, Y_HEADER + 4,
        (uint8_t *)"  STM32 TERRAIN BOT", LEFT_MODE);

    /* Switch to smaller font for data rows */
    BSP_LCD_SetFont(&Font16);

    /* ── Section dividers & static labels ── */
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_IMU_TITLE,
        (uint8_t *)"IMU - ORIENTATION", LEFT_MODE);

    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_ROLL,   (uint8_t *)"Roll  :", LEFT_MODE);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_PITCH,  (uint8_t *)"Pitch :", LEFT_MODE);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_YAW,    (uint8_t *)"Yaw   :", LEFT_MODE);

    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_WHL_TITLE,
        (uint8_t *)"WHEEL VELOCITY", LEFT_MODE);

    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_WHL_LEFT,  (uint8_t *)"Left  :", LEFT_MODE);
    BSP_LCD_DisplayStringAt(MARGIN_X, Y_WHL_RIGHT, (uint8_t *)"Right :", LEFT_MODE);

    /* Separator lines */
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_DrawHLine(0, Y_WHL_TITLE - 2, LCD_W);
    BSP_LCD_DrawHLine(0, Y_BAR - 4,       LCD_W);

    /* Initial bar (zero) */
    draw_hbar(0.0f, 180.0f);
}

void LCD_Display_Update(const Madgwick_t *ahrs, uint8_t calib_done)
{
    char buf[32];

    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetBackColor(COL_BG);

    /* ── Status in header ── */
    BSP_LCD_SetTextColor(COL_HEADER_BG);
    BSP_LCD_FillRect(LCD_W - 110, Y_HEADER, 110, ROW_H);
    BSP_LCD_SetBackColor(COL_HEADER_BG);
    if (calib_done) {
        BSP_LCD_SetTextColor(COL_OK);
        BSP_LCD_DisplayStringAt(LCD_W - 100, Y_HEADER + 4,
            (uint8_t *)"[  RUN  ]", LEFT_MODE);
    } else {
        BSP_LCD_SetTextColor(COL_CALIB);
        BSP_LCD_DisplayStringAt(LCD_W - 100, Y_HEADER + 4,
            (uint8_t *)"[CALIB..]", LEFT_MODE);
    }
    BSP_LCD_SetBackColor(COL_BG);

    /* ── Convert quaternion → Euler (degrees) ── */
    float q0 = ahrs->q0, q1 = ahrs->q1,
          q2 = ahrs->q2, q3 = ahrs->q3;

    float roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                         1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / 3.14159265f);
    float pitch = asinf (2.0f*(q0*q2 - q3*q1))         * (180.0f / 3.14159265f);
    float yaw   = atan2f(2.0f*(q0*q3 + q1*q2),
                         1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / 3.14159265f);

    /* ── IMU values ── */
    /* Erase + redraw value columns */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(MARGIN_X + 80, Y_ROLL,  LCD_W - MARGIN_X - 80, ROW_H);
    BSP_LCD_FillRect(MARGIN_X + 80, Y_PITCH, LCD_W - MARGIN_X - 80, ROW_H);
    BSP_LCD_FillRect(MARGIN_X + 80, Y_YAW,   LCD_W - MARGIN_X - 80, ROW_H);

    BSP_LCD_SetTextColor(COL_VALUE);
    snprintf(buf, sizeof(buf), "%+7.2f deg", (double)roll);
    BSP_LCD_DisplayStringAt(MARGIN_X + 80, Y_ROLL,  (uint8_t *)buf, LEFT_MODE);

    snprintf(buf, sizeof(buf), "%+7.2f deg", (double)pitch);
    BSP_LCD_DisplayStringAt(MARGIN_X + 80, Y_PITCH, (uint8_t *)buf, LEFT_MODE);

    snprintf(buf, sizeof(buf), "%+7.2f deg", (double)yaw);
    BSP_LCD_DisplayStringAt(MARGIN_X + 80, Y_YAW,   (uint8_t *)buf, LEFT_MODE);

    /* ── Wheel velocities ── */
    float vel_l = Encoder_VelMmps(ENC_LEFT);
    float vel_r = Encoder_VelMmps(ENC_RIGHT);

    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(MARGIN_X + 80, Y_WHL_LEFT,  LCD_W - MARGIN_X - 80, ROW_H);
    BSP_LCD_FillRect(MARGIN_X + 80, Y_WHL_RIGHT, LCD_W - MARGIN_X - 80, ROW_H);

    BSP_LCD_SetTextColor(COL_VALUE);
    snprintf(buf, sizeof(buf), "%+7.2f mm/s", (double)vel_l);
    BSP_LCD_DisplayStringAt(MARGIN_X + 80, Y_WHL_LEFT,  (uint8_t *)buf, LEFT_MODE);

    snprintf(buf, sizeof(buf), "%+7.2f mm/s", (double)vel_r);
    BSP_LCD_DisplayStringAt(MARGIN_X + 80, Y_WHL_RIGHT, (uint8_t *)buf, LEFT_MODE);

    /* ── Pitch bar (±180°) ── */
    draw_hbar(pitch, 180.0f);
}
