# ESP32-WROOM — Encoder Co-processor

This folder contains firmware for the **ESP32-WROOM-32** running as an
I2C slave encoder co-processor for the STM32F746G-DISCO terrain bot.

---

## What it does

The ESP32 uses its built-in **hardware PCNT (Pulse Counter)** peripheral
to count quadrature encoder pulses from all 4 RMCS-3070 motors with
zero CPU overhead. It serves the tick counts and wheel velocities to the
STM32 master over I2C on demand.

```
STM32 (master, addr 0x68 IMU already)
  │
  ├── I2C read  → ESP32 (0x30) → 32 bytes: ticks + speeds
  └── I2C write → ESP32 (0x30) →  4 bytes: motor directions
```

---

## Hardware Setup

### Power
```
STM32 3V3 pin  →  ESP32 3V3
STM32 GND      →  ESP32 GND
```

### I2C (shared bus with ICM-20948)
```
STM32 D15 (PB8)  →  ESP32 GPIO22 (SCL)
STM32 D14 (PB9)  →  ESP32 GPIO21 (SDA)

Pull-up resistors: 4.7kΩ from SDA to 3.3V
                   4.7kΩ from SCL to 3.3V
(check if ICM-20948 module already has them)
```

### Encoder connections
```
Motor   CH_A (green wire)   CH_B (white wire)
─────   ─────────────────   ─────────────────
FL      GPIO4               GPIO5
FR      GPIO12              GPIO13
RL      GPIO14              GPIO15
RR      GPIO33              GPIO32

Red  wire → 5V
Black wire → GND
```

---

## I2C Protocol

### STM32 reads 32 bytes (encoder data)
```
Bytes  0-3  : FL ticks  (int32, little-endian)
Bytes  4-7  : FR ticks  (int32, little-endian)
Bytes  8-11 : RL ticks  (int32, little-endian)
Bytes 12-15 : RR ticks  (int32, little-endian)
Bytes 16-19 : FL speed  (float, mm/s)
Bytes 20-23 : FR speed  (float, mm/s)
Bytes 24-27 : RL speed  (float, mm/s)
Bytes 28-31 : RR speed  (float, mm/s)
```

### STM32 writes 4 bytes (direction info)
```
Byte 0 : FL direction (0=forward, 1=reverse)
Byte 1 : FR direction
Byte 2 : RL direction
Byte 3 : RR direction
```

---

## PCNT Quadrature Mode

The ESP32 PCNT counts ×4 quadrature:
```
CH_A rising  + CH_B LOW  → count UP   (forward)
CH_A rising  + CH_B HIGH → count DOWN (reverse)
CH_B rising  + CH_A HIGH → count UP
CH_B rising  + CH_A LOW  → count DOWN
```

This gives 2264 ticks per wheel revolution:
```
PPR (motor shaft) = 11
Gear ratio        = 51.45
Quadrature        = ×4
Ticks per rev     = 11 × 51.45 × 4 = 2264
mm per tick       = 26.70mm / 2264  = 0.01179mm
```

---

## Build & Flash

Uses PlatformIO:

```bash
# Install PlatformIO CLI
pip install platformio

# Build
cd esp32
pio run

# Flash (connect ESP32 via USB)
pio run --target upload

# Monitor serial output
pio device monitor
```

Or use **VS Code + PlatformIO extension**:
1. Open the `esp32/` folder in VS Code
2. Click the PlatformIO upload button

---

## Serial Debug Output

When running, the ESP32 prints every 500ms:
```
FL: +32.1 FR: +31.8 RL: +32.0 RR: +31.5 mm/s | ticks FL:1234 FR:1189 RL:1221 RR:1198
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| STM32 I2C fails | Address conflict | Confirm ESP32 is at 0x30 |
| Encoders count wrong | CH_A/CH_B swapped | Swap green/white wires |
| Speed always 0 | PCNT not started | Check GPIO pin numbers |
| Counts drift | Noise on encoder lines | Increase PCNT filter value |
| I2C timeout | Pull-ups missing | Add 4.7kΩ on SDA and SCL |
