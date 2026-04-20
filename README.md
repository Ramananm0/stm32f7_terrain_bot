# STM32F746G-DISCO Terrain Rover Firmware

Real-time motion controller for a four-wheel skid-steer terrain rover.
The STM32 handles all safety logic, motor control, and ROS communication.
An ESP32 co-processor handles all four wheel encoders via hardware PCNT.

---

## System Overview

```
┌─────────────────────────────────────────────────────────┐
│           STM32F746G-DISCO  (master controller)         │
│                                                         │
│  ICM-20948 IMU  ←── I2C1 (PB8/PB9) ──→ ESP32 (0x30)  │
│  Safety layer   ←── Madgwick AHRS @ 100Hz              │
│  4× BTS7960     ←── 8 PWM pins (4WD individual)        │
│  micro-ROS      ←── USART6 PC6/PC7 → Raspberry Pi      │
│  LCD dashboard  ←── LTDC built-in 480×272               │
└─────────────────────────────────────────────────────────┘
             │ I2C1 shared bus (400kHz)
    ┌────────┴────────┐
    │                 │
┌───▼────┐     ┌──────▼──────────────────────┐
│ICM-    │     │ ESP32-WROOM  I2C slave 0x30 │
│20948   │     │ PCNT FL/FR/RL/RR quadrature │
│0x68    │     │ → ticks + velocity → STM32  │
└────────┘     └─────────────────────────────┘
```

---

## Hardware

### I2C Bus (shared)
```
STM32 PB8 (D15/CN2 pin3) = SCL  ──→ ICM-20948 SCL + ESP32 GPIO22
STM32 PB9 (D14/CN2 pin1) = SDA  ──→ ICM-20948 SDA + ESP32 GPIO21
Pull-ups: 4.7kΩ on SDA and SCL to 3.3V
```

### Motor PWM (all Arduino header, 20 kHz)
```
Motor   RPWM (forward)          LPWM (reverse)
─────   ──────────────────────  ──────────────────────
FL      A4  (PF7 / TIM11_CH1)  A5  (PF6 / TIM10_CH1)
FR      A2  (PF9 / TIM14_CH1)  A3  (PF8 / TIM13_CH1)
RL      D6  (PH6 / TIM12_CH1)  D11 (PB15/ TIM12_CH2)
RR      D10 (PA8 / TIM1_CH1 )  D3  (PB4 / TIM3_CH1 )
```

### Voltage Protection (12V motors, 14V battery max)
```
Max PWM duty = 12V/14V × 10799 = 9256  (85.7%)
Motor always sees ≤ 12V regardless of battery charge state.
Soft start ramp: 200 counts per 10ms → full speed in ~460ms.
```

### BTS7960 wiring (per motor)
```
RPWM → PWM forward pin   R_EN → 3.3V (tie HIGH)
LPWM → PWM reverse pin   L_EN → 3.3V (tie HIGH)
VCC  → 5V                GND  → common GND
RIS  → not connected      LIS  → not connected
B+/B-→ motor terminals   M+/M-→ 12V battery
```

### RMCS-3070 encoder cable
```
Red   → 5V      Black → GND
Green → CH_A    White → CH_B
Yellow→ Motor+  Blue  → Motor-
```

### UART to Raspberry Pi
```
STM32 D1 (PC6) TX → USB-TTL white wire (RXD)
STM32 D0 (PC7) RX → USB-TTL green wire (TXD)
STM32 GND      → USB-TTL black wire (GND)
Red wire       → DO NOT CONNECT
Baud rate: 2,000,000
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
  LCD_Init → ICM20948_Init → Encoder_Init(I2C)
  → Motor_Init(7 timers) → Madgwick_Init
  → uros_setup (blocks until RPi agent)
  → Gyro calibration 200 samples ~2s (keep still!)
  → Seed Madgwick from accel

Main loop tasks:
  100Hz  IMU read → Madgwick AHRS → Safety layer
         risk R = f(pitch, roll, accel shake)
         scale = (1-R)²   E-stop if pitch>30° or roll>40°

  50Hz   Encoder_Update() → I2C read 32 bytes from ESP32

  100Hz  Per-motor PI:
         error = target×scale - encoder_vel
         PWM = target + KP×error + KI×integral

  100Hz  Publish /imu/data

  50Hz   Publish /wheel_ticks + /wheel_velocity

  10Hz   LCD_Display_Update()

  cont.  rclc_executor_spin_some() → process /cmd_vel

cmd_vel kinematics:
  v_left  = linear.x×1000 − angular.z × wheelbase/2
  v_right = linear.x×1000 + angular.z × wheelbase/2
  FL=RL=v_left   FR=RR=v_right  (skid-steer)

Watchdog: 500ms without /cmd_vel → Motor_StopAll()
```

---

## micro-ROS Topics

| Topic | Type | Rate | Direction |
|-------|------|------|-----------|
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | STM32 → RPi |
| `/wheel_ticks` | `std_msgs/Int32MultiArray` | 50 Hz | STM32 → RPi |
| `/wheel_velocity` | `std_msgs/Float32MultiArray` | 50 Hz | STM32 → RPi |
| `/cmd_vel` | `geometry_msgs/Twist` | on demand | RPi → STM32 |

---

## CubeMX Configuration

```
I2C1   : PB8(SCL) PB9(SDA)  Fast Mode 400kHz
USART6 : PC6(TX)  PC7(RX)   2000000 baud 8N1 no flow ctrl

TIM10  : PWM CH1  PF6(A5)   PSC=0 ARR=10799  FL LPWM
TIM11  : PWM CH1  PF7(A4)   PSC=0 ARR=10799  FL RPWM
TIM12  : PWM CH1  PH6(D6)   PSC=0 ARR=10799  RL RPWM
         PWM CH2  PB15(D11) PSC=0 ARR=10799  RL LPWM
TIM13  : PWM CH1  PF8(A3)   PSC=0 ARR=10799  FR LPWM
TIM14  : PWM CH1  PF9(A2)   PSC=0 ARR=10799  FR RPWM
TIM1   : PWM CH1  PA8(D10)  PSC=0 ARR=10799  RR RPWM
TIM3   : PWM CH1  PB4(D3)   PSC=0 ARR=10799  RR LPWM

Clock: HSE bypass 25MHz → PLL M=25 N=432 P=2 → 216MHz
```

---

## LCD Dashboard

The built-in 4.3" display (480×272) shows live at 10 Hz:

```
┌────────────────────────────────────────────────────────────┐
│ TERRAIN BOT v2.0   RISK[████░░] BAT~12.4V      [ RUN ]    │
├───────────────────────┬────────────────────────────────────┤
│ IMU ORIENTATION       │ MOTOR SPEEDS (mm/s)                │
│ Roll  +012.3° ░░bar░░ │ FL ►[████████░░] +32.1            │
│ Pitch -005.7° ░░bar░░ │ FR ►[████████░░] +31.8            │
│ Yaw   +089.1° ░░bar░░ │ RL ►[████████░░] +32.0            │
│                       │ RR ►[████████░░] +31.5            │
│ ACCEL (m/s²)          │ PWM DUTY (max 85.7%)               │
│ X+0.12 Y+0.05 Z+9.81  │ FL [░░░░░░░░░] FR [░░░░░░░░░]    │
├───────────────────────┴────────────────────────────────────┤
│ PITCH SAFETY  [████░░░░░░░░░░░░░░░░░░░░]  -005.7°         │
└────────────────────────────────────────────────────────────┘

Green = safe/forward   Amber = caution >15°   Red = danger >25°/reverse
```

---

## Repository Structure

```
Core/
├── Inc/
│   ├── app.h              Entry point
│   ├── encoder.h          ESP32 I2C encoder reader
│   ├── icm20948.h         ICM-20948 9-DOF IMU driver
│   ├── lcd_display.h      Dashboard API
│   ├── madgwick.h         Madgwick AHRS filter
│   ├── microros_transport.h  USART6 micro-ROS transport
│   └── motor.h            4× BTS7960 4WD driver
└── Src/
    ├── app.c              Main loop, safety, PID, ROS
    ├── encoder.c          I2C read from ESP32
    ├── icm20948.c         IMU + magnetometer driver
    ├── lcd_display.c      Graphical dashboard
    ├── madgwick.c         Quaternion MARG/IMU filter
    ├── microros_transport.c  UART callbacks
    └── motor.c            PWM + voltage protection

esp32/
├── README.md              ESP32 wiring + build guide
├── platformio.ini         PlatformIO config
└── src/
    └── main.cpp           PCNT quadrature + I2C slave

PIN_CONNECTIONS.txt        Complete wiring table
SETUP_GUIDE.md             Step-by-step CubeMX + RPi setup
README.md                  This file
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| LCD stuck on CALIB | micro-ROS agent not running | Start agent on RPi |
| No topics on RPi | UART wiring reversed | Swap D0/D1 wires |
| Motors spin wrong way | RPWM/LPWM swapped | Swap A4↔A5 or A2↔A3 |
| Encoder all zero | ESP32 not responding | Check I2C pull-ups |
| Motor overheats | PWM too high | Verify MOTOR_MAX_DUTY=9256 |
| Robot stops randomly | Safety E-stop | Check IMU orientation |
| I2C address clash | Wrong ESP32 addr | Confirm 0x30 in ESP32 code |
