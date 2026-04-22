#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR 0x30
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 400000

#define ENC_NUM 4
#define REG_READ_LEN 32

static uint8_t g_tx_buf[REG_READ_LEN];
static volatile uint32_t g_request_count;
static volatile uint32_t g_receive_count;
static uint8_t g_last_dirs[ENC_NUM];

static void build_zero_packet(void)
{
    int32_t ticks[ENC_NUM] = {0, 0, 0, 0};
    float speeds[ENC_NUM] = {0.0f, 0.0f, 0.0f, 0.0f};

    memcpy(&g_tx_buf[0], ticks, sizeof(ticks));
    memcpy(&g_tx_buf[16], speeds, sizeof(speeds));
}

static void on_request(void)
{
    g_request_count++;
    Wire.write(g_tx_buf, REG_READ_LEN);
}

static void on_receive(int num_bytes)
{
    g_receive_count++;

    for (int i = 0; i < ENC_NUM && Wire.available(); i++) {
        g_last_dirs[i] = Wire.read();
    }

    while (Wire.available()) {
        Wire.read();
    }

    (void)num_bytes;
}

void setup(void)
{
    Serial.begin(115200);
    delay(200);

    memset(g_last_dirs, 0, sizeof(g_last_dirs));
    build_zero_packet();

    Wire.begin(I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    Wire.onRequest(on_request);
    Wire.onReceive(on_receive);

    Serial.println("ESP32 minimal I2C encoder slave");
    Serial.printf("addr=0x%02X SDA=GPIO%d SCL=GPIO%d\n",
                  I2C_SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
}

void loop(void)
{
    static uint32_t last_print = 0;

    build_zero_packet();

    if (millis() - last_print >= 1000u) {
        last_print = millis();
        Serial.printf("i2c requests=%lu receives=%lu dirs=%u,%u,%u,%u\n",
                      (unsigned long)g_request_count,
                      (unsigned long)g_receive_count,
                      g_last_dirs[0], g_last_dirs[1],
                      g_last_dirs[2], g_last_dirs[3]);
    }

    delay(10);
}
