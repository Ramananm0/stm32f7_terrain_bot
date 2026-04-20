/**
 * lcd_display.c — STM32F746G-DISCO Full Graphical Dashboard
 *
 * Screen layout (480 × 272 px):
 * ┌──────────────────────────────────────────────────────────────┐
 * │  STM32 TERRAIN BOT          RISK:[===   ]      [STATUS]     │ 0-27
 * ├─────────────────────────┬────────────────────────────────────┤
 * │  IMU ORIENTATION        │  WHEEL SPEEDS                      │
 * │  Roll  : +000.0°        │  FL [=========>     ] +00.0 mm/s  │
 * │  ████bar████            │  FR [=========>     ] +00.0 mm/s  │
 * │  Pitch : +000.0°        │  RL [=========>     ] +00.0 mm/s  │
 * │  ████bar████            │  RR [=========>     ] +00.0 mm/s  │
 * │  Yaw   : +000.0°        │                                    │
 * │  ████bar████            │                                    │
 * ├─────────────────────────┴────────────────────────────────────┤
 * │  SAFETY  pitch bar (±30°)  [===========>                  ] │
 * └──────────────────────────────────────────────────────────────┘
 */

#include "lcd_display.h"
#include "encoder.h"
#include <math.h>
#include <stdio.h>

#include "stm32746g_discovery_lcd.h"

/* ── Colours ─────────────────────────────────────────────────────── */
#define COL_BG          LCD_COLOR_BLACK
#define COL_HEADER_BG   0xFF001F4Bu    /* deep navy blue              */
#define COL_HEADER_TXT  LCD_COLOR_WHITE
#define COL_SECTION     0xFF00D4FFu    /* bright cyan                 */
#define COL_LABEL       0xFFAAAAAau    /* grey                        */
#define COL_VALUE       LCD_COLOR_WHITE
#define COL_OK          0xFF00E676u    /* bright green                */
#define COL_WARN        0xFFFFAB00u    /* amber                       */
#define COL_DANGER      0xFFFF1744u    /* red                         */
#define COL_CALIB       0xFFFFAB00u    /* amber                       */
#define COL_DIVIDER     0xFF1E3A5Fu    /* dark blue divider           */

/* Bar colours */
#define COL_BAR_BG      0xFF1A1A2Eu    /* dark background             */
#define COL_BAR_FWD     0xFF00E676u    /* green = forward             */
#define COL_BAR_REV     0xFFFF1744u    /* red   = reverse             */
#define COL_BAR_PITCH   0xFF448AFFu    /* blue  = pitch               */
#define COL_BAR_ROLL    0xFFFF6D00u    /* orange = roll               */
#define COL_BAR_YAW     0xFFE040FBu    /* purple = yaw                */
#define COL_BAR_RISK    0xFFFF1744u    /* red = risk                  */
#define COL_BAR_SAFE    0xFF00E676u    /* green = safe                */

/* ── Layout ──────────────────────────────────────────────────────── */
#define LCD_W           480
#define LCD_H           272

/* Header */
#define HDR_H           26
#define HDR_Y           0

/* Divider between left and right panels */
#define DIV_X           240

/* Left panel: IMU */
#define L_X             8
#define L_W             (DIV_X - L_X - 4)

/* Right panel: Wheel speeds */
#define R_X             (DIV_X + 8)
#define R_W             (LCD_W - R_X - 8)

/* Row heights */
#define SEC_H           18    /* section title height */
#define ROW_H           22    /* data row height      */
#define BAR_H           10    /* graphical bar height */
#define BAR_MARGIN      2     /* bar inner margin     */

/* Left panel Y positions */
#define L_SEC_Y         (HDR_H + 4)
#define L_ROLL_Y        (L_SEC_Y + SEC_H + 2)
#define L_ROLL_BAR_Y    (L_ROLL_Y + ROW_H)
#define L_PITCH_Y       (L_ROLL_BAR_Y + BAR_H + 4)
#define L_PITCH_BAR_Y   (L_PITCH_Y + ROW_H)
#define L_YAW_Y         (L_PITCH_BAR_Y + BAR_H + 4)
#define L_YAW_BAR_Y     (L_YAW_Y + ROW_H)

/* Right panel Y positions */
#define R_SEC_Y         (HDR_H + 4)
#define R_FL_Y          (R_SEC_Y + SEC_H + 4)
#define R_FR_Y          (R_FL_Y + ROW_H + BAR_H + 6)
#define R_RL_Y          (R_FR_Y + ROW_H + BAR_H + 6)
#define R_RR_Y          (R_RL_Y + ROW_H + BAR_H + 6)

/* Bottom safety bar */
#define BOT_BAR_Y       (LCD_H - 20)
#define BOT_BAR_H       14
#define BOT_BAR_X       8
#define BOT_BAR_W       (LCD_W - 16)

/* ── Helpers ─────────────────────────────────────────────────────── */

/* Draw a horizontal progress bar centered at 0 (bidirectional) */
static void draw_bar_bidir(int x, int y, int w, int h,
                            float value, float max_val,
                            uint32_t col_pos, uint32_t col_neg)
{
    /* Background */
    BSP_LCD_SetTextColor(COL_BAR_BG);
    BSP_LCD_FillRect(x, y, w, h);

    /* Centre marker */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DrawVLine(x + w/2, y, h);

    if (max_val == 0.0f) return;
    if (value >  max_val) value =  max_val;
    if (value < -max_val) value = -max_val;

    int half  = w / 2;
    int fill  = (int)(value / max_val * (float)half);

    if (fill > 0) {
        BSP_LCD_SetTextColor(col_pos);
        BSP_LCD_FillRect(x + half, y + BAR_MARGIN,
                         fill, h - BAR_MARGIN*2);
    } else if (fill < 0) {
        BSP_LCD_SetTextColor(col_neg);
        BSP_LCD_FillRect(x + half + fill, y + BAR_MARGIN,
                         -fill, h - BAR_MARGIN*2);
    }
}

/* Draw a horizontal bar 0→max (unidirectional, for speed magnitude) */
static void draw_bar_unidir(int x, int y, int w, int h,
                             float value, float max_val,
                             uint32_t col)
{
    BSP_LCD_SetTextColor(COL_BAR_BG);
    BSP_LCD_FillRect(x, y, w, h);

    if (max_val == 0.0f) return;
    float ratio = fabsf(value) / max_val;
    if (ratio > 1.0f) ratio = 1.0f;

    int fill = (int)(ratio * (float)w);
    if (fill > 0) {
        BSP_LCD_SetTextColor(col);
        BSP_LCD_FillRect(x, y + BAR_MARGIN, fill, h - BAR_MARGIN*2);
    }
}

/* Choose bar colour based on speed direction */
static uint32_t speed_colour(float vel)
{
    if (vel > 0.5f)  return COL_BAR_FWD;
    if (vel < -0.5f) return COL_BAR_REV;
    return COL_LABEL;
}

/* Draw one wheel speed row + bar */
static void draw_wheel(int y, const char *label, float vel_mmps)
{
    char buf[24];
    int  bar_y  = y + ROW_H;
    int  lbl_w  = 28;   /* "FL " label width px */
    int  val_w  = 80;   /* "+00.0 mm/s" value width */
    int  bar_x  = R_X + lbl_w;
    int  bar_w  = R_W - lbl_w - val_w - 4;

    /* Erase row */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(R_X, y, R_W, ROW_H);

    /* Label */
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(R_X, y + 3, (uint8_t *)label, LEFT_MODE);

    /* Value */
    snprintf(buf, sizeof(buf), "%+5.1f", (double)vel_mmps);
    BSP_LCD_SetTextColor(COL_VALUE);
    BSP_LCD_DisplayStringAt(R_X + lbl_w + bar_w + 4, y + 3,
                            (uint8_t *)buf, LEFT_MODE);

    /* Bar — bidirectional, green=fwd, red=rev */
    draw_bar_bidir(bar_x, bar_y, bar_w, BAR_H,
                   vel_mmps, MOTOR_MAX_SPEED_MMPS,
                   COL_BAR_FWD, COL_BAR_REV);
}

/* Colour for IMU angle bars — changes with severity */
static uint32_t angle_colour(float abs_deg, float warn, float danger)
{
    if (abs_deg >= danger) return COL_DANGER;
    if (abs_deg >= warn)   return COL_WARN;
    return COL_OK;
}

/* Draw one IMU row (label + value) + graphical bar below it */
static void draw_imu_row(int y, int bar_y,
                         const char *label, float value_deg,
                         float max_deg, uint32_t bar_col)
{
    char buf[24];

    /* Erase */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(L_X, y, L_W, ROW_H);

    /* Label */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(L_X, y + 3, (uint8_t *)label, LEFT_MODE);

    /* Value */
    snprintf(buf, sizeof(buf), "%+6.1f\xB0", (double)value_deg);
    uint32_t vcol = angle_colour(fabsf(value_deg), 15.0f, 25.0f);
    BSP_LCD_SetTextColor(vcol);
    BSP_LCD_DisplayStringAt(L_X + 60, y + 3, (uint8_t *)buf, LEFT_MODE);

    /* Graphical bar */
    draw_bar_bidir(L_X, bar_y, L_W, BAR_H,
                   value_deg, max_deg, bar_col, bar_col);
}

/* ── Public: Init ────────────────────────────────────────────────── */
void LCD_Display_Init(void)
{
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
    BSP_LCD_DisplayOn();

    /* Full black background */
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(0, 0, LCD_W, LCD_H);

    /* ── Header bar ── */
    BSP_LCD_SetTextColor(COL_HEADER_BG);
    BSP_LCD_FillRect(0, HDR_Y, LCD_W, HDR_H);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_SetBackColor(COL_HEADER_BG);
    BSP_LCD_DisplayStringAt(6, HDR_Y + 5,
        (uint8_t *)"STM32 TERRAIN BOT", LEFT_MODE);

    /* ── Vertical divider ── */
    BSP_LCD_SetTextColor(COL_DIVIDER);
    BSP_LCD_DrawVLine(DIV_X, HDR_H, LCD_H - HDR_H - 18);

    /* ── Horizontal bottom separator ── */
    BSP_LCD_SetTextColor(COL_DIVIDER);
    BSP_LCD_DrawHLine(0, BOT_BAR_Y - 4, LCD_W);

    /* ── Left section title ── */
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetTextColor(COL_SECTION);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(L_X, L_SEC_Y, (uint8_t *)"IMU ORIENTATION", LEFT_MODE);

    /* ── Right section title ── */
    BSP_LCD_DisplayStringAt(R_X, R_SEC_Y, (uint8_t *)"WHEEL SPEEDS (mm/s)", LEFT_MODE);

    /* ── Bottom bar label ── */
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_DisplayStringAt(L_X, BOT_BAR_Y - 14,
        (uint8_t *)"PITCH SAFETY", LEFT_MODE);

    /* Initial empty bars */
    draw_bar_bidir(L_X, L_ROLL_BAR_Y,  L_W, BAR_H, 0, 180, COL_BAR_ROLL, COL_BAR_ROLL);
    draw_bar_bidir(L_X, L_PITCH_BAR_Y, L_W, BAR_H, 0, 180, COL_BAR_PITCH, COL_BAR_PITCH);
    draw_bar_bidir(L_X, L_YAW_BAR_Y,   L_W, BAR_H, 0, 180, COL_BAR_YAW, COL_BAR_YAW);
    draw_bar_bidir(BOT_BAR_X, BOT_BAR_Y, BOT_BAR_W, BOT_BAR_H, 0, 30, COL_BAR_SAFE, COL_BAR_SAFE);
}

/* ── Public: Update ──────────────────────────────────────────────── */
void LCD_Display_Update(const Madgwick_t *ahrs,
                        uint8_t calib_done,
                        float risk_scaled)
{
    char buf[32];

    /* ── Status badge in header ── */
    BSP_LCD_SetFont(&Font12);
    BSP_LCD_SetTextColor(COL_HEADER_BG);
    BSP_LCD_FillRect(LCD_W - 90, HDR_Y, 90, HDR_H);
    BSP_LCD_SetBackColor(COL_HEADER_BG);
    if (calib_done) {
        BSP_LCD_SetTextColor(COL_OK);
        BSP_LCD_DisplayStringAt(LCD_W - 80, HDR_Y + 5,
            (uint8_t *)"[ RUN ]", LEFT_MODE);
    } else {
        BSP_LCD_SetTextColor(COL_CALIB);
        BSP_LCD_DisplayStringAt(LCD_W - 80, HDR_Y + 5,
            (uint8_t *)"[CALIB]", LEFT_MODE);
    }

    /* Risk bar in header */
    int risk_bar_x = 190;
    int risk_bar_w = 120;
    BSP_LCD_SetTextColor(COL_LABEL);
    BSP_LCD_SetBackColor(COL_HEADER_BG);
    BSP_LCD_DisplayStringAt(risk_bar_x - 42, HDR_Y + 5,
        (uint8_t *)"RISK:", LEFT_MODE);
    draw_bar_unidir(risk_bar_x, HDR_Y + 6, risk_bar_w, HDR_H - 10,
                    risk_scaled, 1.0f,
                    risk_scaled > 0.7f ? COL_DANGER :
                    risk_scaled > 0.3f ? COL_WARN : COL_BAR_SAFE);

    BSP_LCD_SetBackColor(COL_BG);

    /* ── Euler angles ── */
    float q0 = ahrs->q0, q1 = ahrs->q1,
          q2 = ahrs->q2, q3 = ahrs->q3;
    float roll  = atan2f(2.0f*(q0*q1 + q2*q3),
                         1.0f - 2.0f*(q1*q1 + q2*q2))
                  * (180.0f / 3.14159265f);
    float pitch = asinf(2.0f*(q0*q2 - q3*q1))
                  * (180.0f / 3.14159265f);
    float yaw   = atan2f(2.0f*(q0*q3 + q1*q2),
                         1.0f - 2.0f*(q2*q2 + q3*q3))
                  * (180.0f / 3.14159265f);

    BSP_LCD_SetFont(&Font12);

    /* Roll row + bar */
    draw_imu_row(L_ROLL_Y,  L_ROLL_BAR_Y,
                 "Roll :", roll,  180.0f, COL_BAR_ROLL);

    /* Pitch row + bar */
    draw_imu_row(L_PITCH_Y, L_PITCH_BAR_Y,
                 "Pitch:", pitch, 180.0f, COL_BAR_PITCH);

    /* Yaw row + bar */
    draw_imu_row(L_YAW_Y,   L_YAW_BAR_Y,
                 "Yaw  :", yaw,   180.0f, COL_BAR_YAW);

    /* ── Wheel speed rows + bars ── */
    draw_wheel(R_FL_Y, "FL", Encoder_VelMmps(ENC_FL));
    draw_wheel(R_FR_Y, "FR", Encoder_VelMmps(ENC_FR));
    draw_wheel(R_RL_Y, "RL", Encoder_VelMmps(ENC_RL));
    draw_wheel(R_RR_Y, "RR", Encoder_VelMmps(ENC_RR));

    /* ── Bottom safety pitch bar (±30° range) ── */
    uint32_t pitch_col = fabsf(pitch) > 25.0f ? COL_DANGER :
                         fabsf(pitch) > 15.0f ? COL_WARN   : COL_BAR_SAFE;
    draw_bar_bidir(BOT_BAR_X, BOT_BAR_Y, BOT_BAR_W, BOT_BAR_H,
                   pitch, 30.0f, pitch_col, pitch_col);

    /* Pitch value next to bottom bar */
    snprintf(buf, sizeof(buf), "%+5.1f\xB0", (double)pitch);
    BSP_LCD_SetTextColor(pitch_col);
    BSP_LCD_SetBackColor(COL_BG);
    BSP_LCD_DisplayStringAt(LCD_W - 55, BOT_BAR_Y - 14,
                            (uint8_t *)buf, LEFT_MODE);
}
