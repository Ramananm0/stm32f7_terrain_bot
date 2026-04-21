# STM32F746G-DISCO Terrain Rover Firmware

Real-time motion controller for a four-wheel skid-steer terrain rover.
The STM32 handles all safety logic, motor control, LCD, and communication.
An ESP32 co-processor handles all four wheel encoders via hardware PCNT over I2C.

---

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│           STM32F746G-DISCO  (master controller)         │
│                                                         │
│  Encoder ESP32  ←── I2C1 (PB8/PB9) @ 400 kHz          │
│  4× BTS7960     ←── 8 PWM pins (4WD individual)        │
│  USART6         ←── PG14/PG9 @ 115200 baud             │
│  LCD dashboard  ←── LTDC built-in 480×272               │
│  SDRAM FB       ←── FMC SDRAM @ 0xC0000000              │
└─────────────────────────────────────────────────────────┘
             │ I2C1 shared bus (400kHz)
             │
    ┌────────▼──────────────────────┐
    │ ESP32-WROOM  I2C slave 0x30   │
    │ PCNT FL/FR/RL/RR quadrature   │
    │ → ticks + velocity → STM32    │
    └───────────────────────────────┘
```

---

## Hardware

### I2C Bus
```
STM32 PB8 = I2C1_SCL  ──→ ESP32 encoder slave SCL
STM32 PB9 = I2C1_SDA  ──→ ESP32 encoder slave SDA
Bus speed : 400 kHz
ESP32 addr: 0x30
```

### Motor PWM (final CubeMX mapping, 20 kHz on all channels)
```
Motor   RPWM (forward)            LPWM (reverse)
─────   ───────────────────────   ───────────────────────
FL      PF7  / TIM11_CH1          PF6  / TIM10_CH1
FR      PF9  / TIM14_CH1          PF8  / TIM13_CH1
RL      PH6  / TIM12_CH1          PB15 / TIM12_CH2
RR      PA15 / TIM2_CH1           PB4  / TIM3_CH1
```

### PWM Frequency Configuration
```
APB2 timers (216 MHz timer clock)
  TIM10, TIM11  : PSC = 0, ARR = 10799  → 20 kHz

APB1 timers (108 MHz timer clock)
  TIM2, TIM3, TIM12, TIM13, TIM14 : PSC = 0, ARR = 5399 → 20 kHz
```

### Voltage Protection (12V motors, battery up to ~14V)
```
Duty cap = 12 / 14 = 0.857 = 85.7%

TIM10 / TIM11 max compare = 10799 × 0.857 = 9254
TIM2 / TIM3 / TIM12 / TIM13 / TIM14 max compare = 5399 × 0.857 = 4626

This keeps effective motor voltage at or below ~12V even near full battery charge.
```

### BTS7960 wiring (per motor)
```
RPWM → PWM forward pin   R_EN → 3.3V (tie HIGH)
LPWM → PWM reverse pin   L_EN → 3.3V (tie HIGH)
VCC  → 5V                GND  → common GND
RIS  → not connected      LIS  → not connected
B+/B-→ motor terminals   M+/M-→ battery supply
```

### RMCS-3070 encoder cable
```
Red   → 5V      Black → GND
Green → CH_A    White → CH_B
Yellow→ Motor+  Blue  → Motor-
```

### USART6
```
STM32 PG14 = USART6_TX
STM32 PG9  = USART6_RX
Baud rate  = 115200
```

---

## ESP32 Co-processor

See `esp32/README.md` for full wiring and build instructions.

### Pin assignment
```
I2C slave  : SDA=GPIO21  SCL=GPIO22  addr=0x30
FL encoder : CH_A=GPIO4   CH_B=GPIO5
FR encoder : CH_A=GPIO12  CH_B=GPIO13
RL encoder : CH_A=GPIO14  CH_B=GPIO15
RR encoder : CH_A=GPIO33  CH_B=GPIO32
Power      : 3.3V from STM32 3V3 pin + common GND
```

### I2C data protocol
```
STM32 reads 32 bytes:
  [0-3]   FL ticks  int32   [16-19] FL speed float mm/s
  [4-7]   FR ticks  int32   [20-23] FR speed float mm/s
  [8-11]  RL ticks  int32   [24-27] RL speed float mm/s
  [12-15] RR ticks  int32   [28-31] RR speed float mm/s

STM32 writes 4 bytes:
  [0] FL direction  [1] FR direction
  [2] RL direction  [3] RR direction  (0=fwd 1=rev)
```

---

## Control Flow

```
Startup:
  LCD_Init → Encoder_Init(I2C)
  → Motor_Init(7 timers)
  → comms init

Main loop tasks:
  50Hz   Encoder_Update() → I2C read 32 bytes from ESP32

  100Hz  Per-motor control:
         error = target - encoder_vel
         PWM = command + KP×error + KI×integral
         clamp to per-timer max compare values

  10Hz   LCD_Display_Update()

cmd_vel kinematics:
  v_left  = linear.x×1000 − angular.z × wheelbase/2
  v_right = linear.x×1000 + angular.z × wheelbase/2
  FL=RL=v_left   FR=RR=v_right  (skid-steer)

Watchdog: timeout without command → Motor_StopAll()
```

---

## Communication Topics / Interfaces

| Interface | Purpose | Direction |
|----------|---------|-----------|
| `I2C1` | Encoder data exchange with ESP32 | STM32 ↔ ESP32 |
| `USART6` | Host / ROS / debug communication | STM32 ↔ external host |
| `LTDC + FMC SDRAM` | LCD framebuffer output | STM32 → LCD |

---

## Final CubeMX Configuration

```
Clock:
  HSE bypass 25MHz → PLL → SYSCLK 216MHz
  APB1 = 54MHz
  APB2 = 108MHz

USART6:
  PG14(TX) / PG9(RX)
  115200 baud

I2C1:
  PB8(SCL) / PB9(SDA)
  400kHz

PWM:
  TIM11_CH1 → PF7   FL RPWM   PSC=0 ARR=10799
  TIM10_CH1 → PF6   FL LPWM   PSC=0 ARR=10799
  TIM14_CH1 → PF9   FR RPWM   PSC=0 ARR=5399
  TIM13_CH1 → PF8   FR LPWM   PSC=0 ARR=5399
  TIM12_CH1 → PH6   RL RPWM   PSC=0 ARR=5399
  TIM12_CH2 → PB15  RL LPWM   PSC=0 ARR=5399
  TIM2_CH1  → PA15  RR RPWM   PSC=0 ARR=5399
  TIM3_CH1  → PB4   RR LPWM   PSC=0 ARR=5399

FMC SDRAM:
  Bank1, 8 column, 12 row, CAS 3, 16-bit

LTDC:
  RGB565, 480×272
  framebuffer = 0xC0000000

Encoder:
  ESP32 via I2C1 at address 0x30
```

---

## LCD Dashboard

The built-in 4.3" display (480×272) shows live status information.

```
┌────────────────────────────────────────────────────────────┐
│ TERRAIN BOT        PWM / ENCODER / LINK STATUS            │
├───────────────────────┬────────────────────────────────────┤
│ COMM STATUS           │ MOTOR SPEEDS                       │
│ USART6 : OK           │ FL  value / duty / dir            │
│ I2C1   : OK           │ FR  value / duty / dir            │
│ ESP32  : 0x30         │ RL  value / duty / dir            │
│                       │ RR  value / duty / dir            │
├───────────────────────┴────────────────────────────────────┤
│ DUTY CAPS: APB2=9254   APB1=4626                          │
└────────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
Core/
├── Inc/
│   ├── app.h
│   ├── encoder.h
│   ├── lcd_display.h
│   └── motor.h
└── Src/
    ├── app.c
    ├── encoder.c
    ├── lcd_display.c
    └── motor.c

esp32/
├── README.md
├── platformio.ini
└── src/
    └── main.cpp

PIN_CONNECTIONS.txt
SETUP_GUIDE.md
README.md
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| No encoder data | ESP32 not responding | Check I2C1 wiring and address `0x30` |
| Motors spin wrong way | RPWM/LPWM swapped | Swap motor direction pins in wiring or software |
| Motor overheats | PWM too high | Verify APB2 cap = `9254`, APB1 cap = `4626` |
| Wrong PWM frequency | Same ARR used on all timers | Use `10799` for TIM10/11 and `5399` for TIM2/3/12/13/14 |
| No host comms | USART6 pins mismatched | Verify PG14/PG9 and 115200 baud |
| LCD corruption | SDRAM/LTDC config wrong | Recheck FMC SDRAM + framebuffer `0xC0000000` |
