/**
 * esp32_encoder_slave — Quadrature Encoder Co-processor
 * Board : ESP32-WROOM-32
 *
 * ── What this does ───────────────────────────────────────────────────
 *   Runs as I2C slave (address 0x30) connected to STM32F746G-DISCO.
 *   Uses ESP32 hardware PCNT (Pulse Counter) for full quadrature
 *   counting on all 4 RMCS-3070 motor encoders.
 *
 *   STM32 READS  → 32 bytes: ticks(×4 int32) + speed(×4 float mm/s)
 *   STM32 WRITES → 4 bytes:  direction per motor (0=fwd, 1=rev)
 *
 * ── Pin Assignment ───────────────────────────────────────────────────
 *   I2C slave:
 *     SDA = GPIO21   (connect to STM32 D14 / PB9)
 *     SCL = GPIO22   (connect to STM32 D15 / PB8)
 *
 *   Encoders (RMCS-3070 — green=CH_A, white=CH_B):
 *     FL CH_A = GPIO4   FL CH_B = GPIO5
 *     FR CH_A = GPIO12  FR CH_B = GPIO13
 *     RL CH_A = GPIO14  RL CH_B = GPIO15
 *     RR CH_A = GPIO33  RR CH_B = GPIO32
 *
 * ── Power ────────────────────────────────────────────────────────────
 *   3V3  ← STM32 3.3V pin
 *   GND  ← common ground with STM32
 *
 * ── PCNT quadrature mode ─────────────────────────────────────────────
 *   CH_A is the pulse input, CH_B is the control input.
 *   Count UP   when CH_A rises and CH_B = LOW  (forward)
 *   Count DOWN when CH_A rises and CH_B = HIGH (reverse)
 *   This gives automatic direction detection from encoder signals.
 *
 * ── RMCS-3070 specs ──────────────────────────────────────────────────
 *   PPR (motor shaft) = 11
 *   Gear ratio        = 51.45
 *   Ticks per rev     = 11 × 51.45 × 4 = 2264 (quadrature ×4)
 *   Wheel circ        = 26.70 mm
 *   mm per tick       = 26.70 / 2264 = 0.01179 mm
 */

#include <Arduino.h>
#include <Wire.h>
#include "driver/pcnt.h"

/* ── I2C config ──────────────────────────────────────────────────── */
#define I2C_SLAVE_ADDR   0x30
#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define I2C_FREQ         400000

/* ── Encoder pins ────────────────────────────────────────────────── */
#define ENC_FL_A   4
#define ENC_FL_B   5
#define ENC_FR_A   12
#define ENC_FR_B   13
#define ENC_RL_A   14
#define ENC_RL_B   15
#define ENC_RR_A   33
#define ENC_RR_B   32

/* ── Encoder indices ─────────────────────────────────────────────── */
#define ENC_FL     0
#define ENC_FR     1
#define ENC_RL     2
#define ENC_RR     3
#define ENC_NUM    4

/* ── Physical constants ──────────────────────────────────────────── */
#define TICKS_PER_REV    2264       /* 11 × 51.45 × 4 quadrature     */
#define WHEEL_CIRC_MM    26.70f
#define MM_PER_TICK      (WHEEL_CIRC_MM / (float)TICKS_PER_REV)

/* PCNT 16-bit counter overflows at ±32767 — use accumulator */
#define PCNT_HIGH_LIMIT  32000
#define PCNT_LOW_LIMIT  -32000

/* ── Speed update interval ───────────────────────────────────────── */
#define SPEED_UPDATE_MS  50         /* calculate speed every 50ms     */

/* ── I2C register map ────────────────────────────────────────────── */
/* READ  (32 bytes): ticks[4]×int32 + speed[4]×float                 */
/* WRITE (4 bytes):  direction[4]×uint8                               */
#define REG_READ_LEN    32
#define REG_WRITE_LEN   4

/* ── State ───────────────────────────────────────────────────────── */
struct EncoderState {
    volatile int32_t  total_ticks;    /* accumulated ticks            */
    volatile float    vel_mmps;       /* speed in mm/s                */
    int16_t           last_count;     /* previous PCNT value          */
    int32_t           accumulator;    /* overflow-safe accumulator     */
    uint8_t           direction;      /* 0=fwd 1=rev from STM32       */
    uint32_t          last_ms;        /* last speed update time        */
    int32_t           last_ticks;     /* ticks at last speed update    */
};

static EncoderState g_enc[ENC_NUM];

/* I2C response buffer — updated atomically */
static uint8_t  g_tx_buf[REG_READ_LEN];
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

/* ── PCNT setup ──────────────────────────────────────────────────── */
static const int enc_a_pins[ENC_NUM] = {ENC_FL_A, ENC_FR_A, ENC_RL_A, ENC_RR_A};
static const int enc_b_pins[ENC_NUM] = {ENC_FL_B, ENC_FR_B, ENC_RL_B, ENC_RR_B};

static void pcnt_setup(int unit, int gpio_a, int gpio_b)
{
    pcnt_config_t cfg;
    cfg.pulse_gpio_num  = gpio_a;
    cfg.ctrl_gpio_num   = gpio_b;
    cfg.unit            = (pcnt_unit_t)unit;
    cfg.channel         = PCNT_CHANNEL_0;
    cfg.pos_mode        = PCNT_COUNT_INC;   /* CH_A rising + CH_B LOW  → UP   */
    cfg.neg_mode        = PCNT_COUNT_DEC;   /* CH_A falling + CH_B LOW → DOWN */
    cfg.lctrl_mode      = PCNT_MODE_REVERSE;/* CH_B HIGH → reverse direction  */
    cfg.hctrl_mode      = PCNT_MODE_KEEP;
    cfg.counter_h_lim   = PCNT_HIGH_LIMIT;
    cfg.counter_l_lim   = PCNT_LOW_LIMIT;

    pcnt_unit_config(&cfg);

    /* Add CH_B as second channel for X4 quadrature */
    cfg.pulse_gpio_num  = gpio_b;
    cfg.ctrl_gpio_num   = gpio_a;
    cfg.channel         = PCNT_CHANNEL_1;
    cfg.pos_mode        = PCNT_COUNT_DEC;
    cfg.neg_mode        = PCNT_COUNT_INC;
    cfg.lctrl_mode      = PCNT_MODE_KEEP;
    cfg.hctrl_mode      = PCNT_MODE_REVERSE;
    pcnt_unit_config(&cfg);

    /* Filter: reject pulses shorter than 1µs (noise) */
    pcnt_set_filter_value((pcnt_unit_t)unit, 80); /* 80 APB cycles @ 80MHz = 1µs */
    pcnt_filter_enable((pcnt_unit_t)unit);

    pcnt_counter_pause((pcnt_unit_t)unit);
    pcnt_counter_clear((pcnt_unit_t)unit);
    pcnt_counter_resume((pcnt_unit_t)unit);
}

/* ── Read PCNT with overflow accumulation ────────────────────────── */
static int32_t pcnt_read_accumulated(int unit)
{
    int16_t count = 0;
    pcnt_get_counter_value((pcnt_unit_t)unit, &count);

    int16_t delta = count - g_enc[unit].last_count;

    /* Detect overflow wrap */
    if (delta > PCNT_HIGH_LIMIT / 2)
        delta -= (int16_t)(PCNT_HIGH_LIMIT - PCNT_LOW_LIMIT);
    else if (delta < PCNT_LOW_LIMIT / 2)
        delta += (int16_t)(PCNT_HIGH_LIMIT - PCNT_LOW_LIMIT);

    g_enc[unit].last_count    = count;
    g_enc[unit].accumulator  += delta;
    g_enc[unit].total_ticks  += delta;

    return g_enc[unit].total_ticks;
}

/* ── Update speed calculation ────────────────────────────────────── */
static void update_speed(int i)
{
    uint32_t now = millis();
    uint32_t dt_ms = now - g_enc[i].last_ms;
    if (dt_ms < SPEED_UPDATE_MS) return;

    int32_t current_ticks = g_enc[i].total_ticks;
    int32_t delta = current_ticks - g_enc[i].last_ticks;

    float dt_s = (float)dt_ms * 0.001f;
    g_enc[i].vel_mmps = (float)delta * MM_PER_TICK / dt_s;

    g_enc[i].last_ticks = current_ticks;
    g_enc[i].last_ms    = now;
}

/* ── Build I2C TX buffer ─────────────────────────────────────────── */
static void build_tx_buffer(void)
{
    portENTER_CRITICAL(&g_mux);

    /* ticks[4] as int32 little-endian */
    for (int i = 0; i < ENC_NUM; i++) {
        int32_t t = g_enc[i].total_ticks;
        memcpy(&g_tx_buf[i * 4], &t, 4);
    }

    /* speed[4] as float little-endian */
    for (int i = 0; i < ENC_NUM; i++) {
        float v = g_enc[i].vel_mmps;
        memcpy(&g_tx_buf[16 + i * 4], &v, 4);
    }

    portEXIT_CRITICAL(&g_mux);
}

/* ── I2C slave callbacks ─────────────────────────────────────────── */
static void on_request(void)
{
    /* STM32 is reading — send 32 bytes */
    Wire.write(g_tx_buf, REG_READ_LEN);
}

static void on_receive(int num_bytes)
{
    if (num_bytes < REG_WRITE_LEN) return;

    /* STM32 is writing direction bytes */
    for (int i = 0; i < ENC_NUM && Wire.available(); i++) {
        uint8_t dir = Wire.read();
        g_enc[i].direction = dir;
    }
    /* Drain any extra bytes */
    while (Wire.available()) Wire.read();
}

/* ── Setup ───────────────────────────────────────────────────────── */
void setup(void)
{
    Serial.begin(115200);
    Serial.println("ESP32 Encoder Slave starting...");

    /* Init encoder state */
    memset(g_enc, 0, sizeof(g_enc));
    for (int i = 0; i < ENC_NUM; i++) {
        g_enc[i].last_ms = millis();
    }

    /* Setup PCNT for all 4 encoders */
    for (int i = 0; i < ENC_NUM; i++) {
        pcnt_setup(i, enc_a_pins[i], enc_b_pins[i]);
        Serial.printf("PCNT%d: CH_A=GPIO%d CH_B=GPIO%d\n",
                       i, enc_a_pins[i], enc_b_pins[i]);
    }

    /* Zero TX buffer */
    memset(g_tx_buf, 0, sizeof(g_tx_buf));

    /* Start I2C slave */
    Wire.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    Wire.onRequest(on_request);
    Wire.onReceive(on_receive);

    Serial.printf("I2C slave ready at 0x%02X on SDA=GPIO%d SCL=GPIO%d\n",
                  I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
}

/* ── Main loop ───────────────────────────────────────────────────── */
void loop(void)
{
    /* Update all encoder accumulators and speeds */
    for (int i = 0; i < ENC_NUM; i++) {
        pcnt_read_accumulated(i);
        update_speed(i);
    }

    /* Rebuild TX buffer for next I2C read */
    build_tx_buffer();

    /* Debug output every 500ms */
    static uint32_t last_dbg = 0;
    if (millis() - last_dbg > 500) {
        last_dbg = millis();
        Serial.printf("FL:%+6.1f FR:%+6.1f RL:%+6.1f RR:%+6.1f mm/s | ticks FL:%d FR:%d RL:%d RR:%d\n",
            g_enc[0].vel_mmps, g_enc[1].vel_mmps,
            g_enc[2].vel_mmps, g_enc[3].vel_mmps,
            g_enc[0].total_ticks, g_enc[1].total_ticks,
            g_enc[2].total_ticks, g_enc[3].total_ticks);
    }

    delay(5);  /* 200Hz update rate */
}
