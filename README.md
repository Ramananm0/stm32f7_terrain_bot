# STM32F746G-DISCO — Terrain Bot Firmware

Real-time motor control, IMU sensing, encoder odometry and ROS2 bridge
for a **4-wheel skid-steer terrain rover**.

Board: **STM32F746NGH6** (Cortex-M7 @ 216 MHz, hardware FPU, 1 MB Flash)
Drive: **Skid-steer / tank drive** — 2× BTS7960 H-bridge drivers
Sensors: ICM-20948 9-DOF IMU + 4× RMCS-3070 wheel encoders
Display: Built-in 4.3" LCD (480×272) — live graphical dashboard
ROS2: micro-ROS over USART6 → Raspberry Pi 4B

---

## Hardware Design Decisions

### Why skid-steer with 2× BTS7960?
Each BTS7960 drives one side (left or right) with two motors wired in
parallel. This gives full 4WD traction with only 4 PWM signals total,
all fitting on the Arduino header with no Morpho connector needed.

### Why Arduino header only?
The STM32F746G-DISCO exposes only Arduino-style headers (D0–D15, A0–A5)
externally. The Morpho connector holes are not populated on this board
revision. All firmware is designed around the 22 available Arduino pins.

### Why interrupt-based encoders (not quadrature timer mode)?
Hardware quadrature timer mode requires both CH1 and CH2 of the same
timer on accessible pins. On this board only TIM12 has both channels on
the Arduino header. The other three timers have CH2 on unpopulated
Morpho pads or internally blocked pins (ETH, LTDC, UART).

Using GPIO interrupts on CH_A only (1 pin per motor) gives accurate
speed measurement for all 4 motors. Direction is inferred from the
BTS7960 command since we control it directly.

### Why separate encoder update rate (10 Hz) vs PID rate (100 Hz)?
Interrupt pulse counts accumulate continuously. Speed is calculated
every 100 ms (10 Hz) from the accumulated count — this gives stable
velocity estimates even at slow terrain speeds. The PID runs at 100 Hz
using the most recent speed estimate.

### Phase 2 — MCP23017 (when it arrives)
The CJMCU-2317 MCP23017 I2C GPIO expander (16 pins over I2C) will add
full quadrature support for all 4 encoders via software interrupt
counting on GPA/GPB pins, improving odometry accuracy for SLAM.

---

## System Architecture

```
╔══════════════════════════════════════════════════════════════════╗
║              STM32F746G-DISCO  (real-time controller)           ║
║                                                                  ║
║  I2C1 (CN2 connector)                                           ║
║    PB8 (SCL/D15) ──────► ICM-20948 IMU                        ║
║    PB9 (SDA/D14)           accel + gyro + AK09916 magnetometer ║
║                             Madgwick AHRS @ 100 Hz              ║
║                                                                  ║
║  EXTI GPIO interrupts (CH_A pulse counting)                     ║
║    PA0  (A0)  ──────► FL encoder   (Front-Left)                ║
║    PB4  (D3)  ──────► FR encoder   (Front-Right)               ║
║    PA15 (D9)  ──────► RL encoder   (Rear-Left)                 ║
║    PG6  (D2)  ──────► RR encoder   (Rear-Right)                ║
║                                                                  ║
║  PWM outputs (20 kHz)                                           ║
║    PF7 (A4) TIM11 RPWM ──► BTS7960 #1 LEFT ──► FL + RL motors ║
║    PF6 (A5) TIM10 LPWM ──►                                     ║
║    PF9 (A2) TIM14 RPWM ──► BTS7960 #2 RIGHT ──► FR + RR motors║
║    PF8 (A3) TIM13 LPWM ──►                                     ║
║                                                                  ║
║  ┌──────────────────────────────────────────────────────────┐   ║
║  │  TERRAIN SAFETY LAYER  (100 Hz, paper §III)             │   ║
║  │                                                          │   ║
║  │  Eq.5:  θ_w = 0.7×|pitch| + 0.3×|roll|                 │   ║
║  │  Eq.9:  R_incl = clamp((θ_w−10°)/(30°−10°), 0,1)       │   ║
║  │  Eq.6:  R_shake = |‖a‖ − g_calib| / g_calib            │   ║
║  │  Eq.8:  R_raw = 0.7×R_incl + 0.3×R_shake               │   ║
║  │  Eq.10: R_f = 0.90×R_f_prev + 0.10×R_raw               │   ║
║  │  Eq.11: speed_scale = (1 − R_f)²                        │   ║
║  │  Eq.12: pitch>30° OR roll>40° → E-stop immediately      │   ║
║  └──────────────────────────────────────────────────────────┘   ║
║                                                                  ║
║  USART6  PC6(TX/D1)  PC7(RX/D0)  2 Mbaud                      ║
║  ◄──────────────────────────────────────────────────────────    ║
║                  micro-ROS custom UART transport                 ║
╚══════════════════════════════════════════════════════════════════╝
                         │ USB-TTL cable
                         │ White→D1  Green→D0  Black→GND
                         ▼
╔══════════════════════════════════════════════════════════════════╗
║              Raspberry Pi 4B  (ROS2 Humble)                     ║
║                                                                  ║
║  micro_ros_agent  ←──►  STM32 node "terrain_bot/stm32f7_node"  ║
║                                                                  ║
║  Subscribes from STM32:                                         ║
║    /imu/data          sensor_msgs/Imu            @ 100 Hz       ║
║    /wheel_ticks       std_msgs/Int32MultiArray   @  50 Hz       ║
║    /wheel_velocity    std_msgs/Float32MultiArray @  50 Hz       ║
║                                                                  ║
║  Publishes to STM32:                                            ║
║    /cmd_vel           geometry_msgs/Twist  (from Nav2/SLAM)     ║
╚══════════════════════════════════════════════════════════════════╝
```

---

## Code Flow

### Startup Sequence (App_Run)

```
Power ON
  │
  ├─ LCD_Display_Init()
  │    Draw static dashboard frame
  │    Show [CALIB..] status
  │
  ├─ ICM20948_Init(&hi2c1)
  │    Soft reset → WHO_AM_I check (0xEA)
  │    Configure accel ±4g @ 102 Hz
  │    Configure gyro ±500dps @ 102 Hz
  │    Start AK09916 magnetometer @ 100 Hz
  │    Error_Handler() if not found
  │
  ├─ Encoder_Init()
  │    Clear all pulse counts
  │    Set all directions = forward
  │
  ├─ Motor_Init(&htim10, &htim11, &htim13, &htim14)
  │    Start TIM10/11/13/14 PWM outputs
  │    Set all duty = 0 (stopped)
  │
  ├─ Madgwick_Init(&ahrs, 0.033)
  │    Identity quaternion [1,0,0,0]
  │
  ├─ uros_setup()
  │    Set UART custom transport
  │    BLOCK until micro-ROS agent responds on RPi
  │    Create node "stm32f7_node" in namespace "terrain_bot"
  │    Advertise /imu/data, /wheel_ticks, /wheel_velocity
  │    Subscribe /cmd_vel
  │
  ├─ Gyro calibration loop (~2 seconds)
  │    Read IMU 200 times @ 100 Hz
  │    Accumulate gx,gy,gz,gravity
  │    Compute bias averages
  │    LCD shows [CALIB..] during this
  │    ROBOT MUST BE STILL!
  │
  ├─ Seed Madgwick with accel gravity vector
  │    Compute initial roll/pitch from accel
  │    Set quaternion from roll/pitch (skip yaw)
  │    ahrs.initialised = true
  │
  └─ Enter main loop
```

### Main Loop (runs forever @ bare-metal polling)

```
while(1):
  │
  ├─ [every 10ms = 100 Hz] IMU task
  │    ICM20948_Read() → accel, gyro, mag
  │    Apply gyro bias correction
  │    if mag valid → Madgwick_UpdateMARG() [9-DOF]
  │    else         → Madgwick_UpdateIMU()  [6-DOF]
  │    update_safety():
  │      Compute θ_w from pitch + roll (Eq.5)
  │      Normalize to R_incl (Eq.9)
  │      Compute R_shake from accel magnitude (Eq.6)
  │      Combine → R_raw (Eq.8)
  │      Low-pass filter → R_filtered (Eq.10)
  │      speed_scale = (1-R)² (Eq.11)
  │      If pitch>30° or roll>40° → Motor_StopAll() (Eq.12)
  │
  ├─ [every 100ms = 10 Hz] Encoder task
  │    Encoder_Update():
  │      For each encoder:
  │        Snapshot pulse_count (disable IRQ briefly)
  │        speed = pulses × mm_per_tick / dt
  │        Apply direction sign
  │        Accumulate distance
  │
  ├─ [every 10ms = 100 Hz] PID task
  │    Check cmd_vel watchdog (500ms timeout)
  │    If timeout → Motor_StopAll()
  │    Else → motor_pid_update():
  │      For LEFT and RIGHT side:
  │        target   = g_target_mmps[side] × speed_scale
  │        measured = avg(FL+RL) or avg(FR+RR) encoder vel
  │        error    = target - measured
  │        integral += error × dt  (anti-windup ±22.25)
  │        cmd = target + KP×error + KI×integral
  │        Motor_SetSide(side, cmd):
  │          vel>0 → RPWM=duty, LPWM=0  (forward)
  │          vel<0 → RPWM=0,    LPWM=duty (reverse)
  │          Encoder_SetDirection() updated
  │
  ├─ [every 10ms = 100 Hz] Publish /imu/data
  │    Quaternion + angular velocity + linear accel
  │    Covariance matrices from ICM-20948 datasheet
  │
  ├─ [every 20ms = 50 Hz] Publish /wheel_ticks + /wheel_velocity
  │    All 4 encoder tick counts and velocities
  │
  ├─ [every 100ms = 10 Hz] LCD update
  │    Roll/Pitch/Yaw text + colour-coded bars
  │    FL/FR/RL/RR wheel speed bars (green=fwd, red=rev)
  │    Risk meter in header
  │    Safety pitch bar at bottom
  │
  └─ rclc_executor_spin_some()
       Process incoming /cmd_vel if available
       cmd_vel_callback():
         vx = linear.x × 1000  (m/s → mm/s)
         wz = angular.z         (rad/s)
         left  = vx - wz × wheelbase/2
         right = vx + wz × wheelbase/2
         Reset watchdog timer
```

### Encoder ISR Flow

```
Rising edge on encoder CH_A pin
  │
  ├─ EXTI IRQ fires
  ├─ HAL_GPIO_EXTI_IRQHandler()
  ├─ HAL_GPIO_EXTI_Callback(GPIO_Pin)
  ├─ Encoder_PulseISR(enc_id)
  │    g_enc[id].pulse_count++
  │    g_enc[id].total_ticks++
  └─ Return (< 1 µs total)

Every 100ms — Encoder_Update():
  speed_mmps = pulse_count × 0.04717mm / 0.1s
  vel_mmps   = speed_mmps × direction (+1 or -1)
  dist_mm   += pulses × 0.04717 × direction
  pulse_count = 0  (reset for next window)
```

---

## File Structure

```
Core/
├── Inc/
│   ├── app.h                  Entry point declaration
│   ├── encoder.h              4-wheel interrupt encoder (1 pin/motor)
│   ├── icm20948.h             ICM-20948 9-DOF IMU driver
│   ├── lcd_display.h          Graphical dashboard API
│   ├── madgwick.h             Madgwick AHRS filter (IEEE ICORR 2011)
│   ├── microros_transport.h   USART6 micro-ROS transport
│   └── motor.h                2× BTS7960 skid-steer driver
│
└── Src/
    ├── app.c                  Main loop, safety layer, PID, ROS bridge
    ├── encoder.c              Pulse counting via EXTI interrupts
    ├── icm20948.c             I2C driver + AK09916 magnetometer
    ├── lcd_display.c          Full graphical LCD dashboard
    ├── madgwick.c             Quaternion MARG/IMU filter
    ├── microros_transport.c   UART send/receive callbacks
    └── motor.c                BTS7960 RPWM/LPWM PWM control

PIN_CONNECTIONS.txt            Complete wiring table
SETUP_GUIDE.md                 Step-by-step CubeMX + RPi setup
README.md                      This file
```

---

## Pin Assignment (Arduino Header Only)

| Arduino | STM32 | Function | Timer/Periph |
|---------|-------|----------|--------------|
| A0 | PA0 | FL encoder CH_A | EXTI0 |
| A2 | PF9 | RIGHT BTS RPWM (forward) | TIM14_CH1 |
| A3 | PF8 | RIGHT BTS LPWM (reverse) | TIM13_CH1 |
| A4 | PF7 | LEFT BTS RPWM (forward) | TIM11_CH1 |
| A5 | PF6 | LEFT BTS LPWM (reverse) | TIM10_CH1 |
| D0 | PC7 | UART RX ← RPi | USART6_RX |
| D1 | PC6 | UART TX → RPi | USART6_TX |
| D2 | PG6 | RR encoder CH_A | EXTI6 |
| D3 | PB4 | FR encoder CH_A | EXTI4 |
| D9 | PA15 | RL encoder CH_A | EXTI15 |
| D14 | PB9 | I2C SDA (IMU) | I2C1_SDA |
| D15 | PB8 | I2C SCL (IMU) | I2C1_SCL |
| CN2 | — | I2C extension (IMU connector) | I2C1 |

**Free pins (available for expansion):**
D4(PG7), D5(PI0), D6(PH6), D7(PI3), D8(PI2), D10(PA8),
D11(PB15), D12(PB14), A1(PF10)

---

## Hardware Connections

### BTS7960 Wiring

```
BTS7960 #1 (LEFT)          BTS7960 #2 (RIGHT)
─────────────────          ──────────────────
RPWM ← A4 (PF7)           RPWM ← A2 (PF9)
LPWM ← A5 (PF6)           LPWM ← A3 (PF8)
R_EN → 3.3V (always HIGH)  R_EN → 3.3V
L_EN → 3.3V (always HIGH)  L_EN → 3.3V
VCC  → 5V                  VCC  → 5V
GND  → GND                 GND  → GND
B+/B- → FL + RL motors     B+/B- → FR + RR motors
M+/M- → 12V supply         M+/M- → 12V supply
```

BTS7960 control logic:
```
Forward: RPWM = PWM duty,  LPWM = 0
Reverse: RPWM = 0,          LPWM = PWM duty
Stop:    RPWM = 0,          LPWM = 0
```

### RMCS-3070 Encoder Wiring

```
Wire Colour   Connect To
Red        →  5V
Black      →  GND
Green      →  CH_A pin (A0/D3/D9/D2 per motor)
White      →  not connected (CH_B, future use)
Yellow     →  BTS7960 motor output +
Blue       →  BTS7960 motor output -
```

### ICM-20948 via CN2

```
CN2 Pin 1 → I2C_SDA (PB9)
CN2 Pin 3 → I2C_SCL (PB8)
CN2 Pin 5 → +3V3
CN2 Pin 7 → GND
ICM AD0   → GND (address = 0x68)
```

### USB-TTL Cable to Raspberry Pi

```
White (RXD) → D1 (PC6)   STM32 TX
Green (TXD) → D0 (PC7)   STM32 RX
Black (GND) → GND
Red/Yellow/Blue → DO NOT CONNECT
```

---

## Safety Layer

Based on:
> Mourougane R., Dhanalakshmi S. *"Terrain-Aware Velocity Regulation for
> Mobile Robots via Lightweight Execution-Level Safety Layer."*
> SRM Institute of Science and Technology.

Runs entirely on STM32 at 100 Hz — no ROS latency involved.

| Zone | Pitch | speed_scale | Behaviour |
|------|-------|-------------|-----------|
| Safe | 0–10° | 1.00 | Full commanded speed |
| Caution | 10–30° | (1−R)² curve | Progressive deceleration |
| E-stop | >30° | 0.00 | Immediate motor stop |

---

## LCD Dashboard

The built-in 4.3" display (480×272, back side of board) shows live data:

```
┌──────────────────────────────────────────────────────────────┐
│ STM32 TERRAIN BOT    RISK:[████░░░░]            [ RUN ]      │
├───────────────────────┬──────────────────────────────────────┤
│ IMU ORIENTATION       │ WHEEL SPEEDS (mm/s)                  │
│ Roll : +012.3°        │ FL [████████████░] +32.1             │
│ ░░orange bar░░░       │ FR [████████░░░░░] +22.5             │
│ Pitch: -005.7°        │ RL [████████████░] +31.8             │
│ ░░blue bar░░░░        │ RR [████████░░░░░] +22.1             │
│ Yaw  : +089.1°        │                                      │
│ ░░purple bar░░        │                                      │
├───────────────────────┴──────────────────────────────────────┤
│ PITCH SAFETY   [████░░░░░░░░░░░░░░░░]   -005.7°             │
└──────────────────────────────────────────────────────────────┘
```

Colour coding:
- 🟢 Green = safe / forward
- 🟡 Amber = caution (>15° or risk >30%)
- 🔴 Red = danger (>25° or risk >70%) / reverse motion
- 🔵 Blue = pitch bar
- 🟠 Orange = roll bar
- 🟣 Purple = yaw bar

---

## micro-ROS Topics

### Published (STM32 → ROS2)

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | Quaternion, angular vel, linear accel |
| `/wheel_ticks` | `std_msgs/Int32MultiArray` | 50 Hz | [FL,FR,RL,RR] total pulse counts |
| `/wheel_velocity` | `std_msgs/Float32MultiArray` | 50 Hz | [FL,FR,RL,RR] speed in mm/s |

### Subscribed (ROS2 → STM32)

| Topic | Type | Content |
|-------|------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` [m/s], `angular.z` [rad/s] |

Kinematics:
```
v_left  = linear.x × 1000 − angular.z × (wheelbase/2)   [mm/s]
v_right = linear.x × 1000 + angular.z × (wheelbase/2)   [mm/s]
```

---

## Encoder Specifications

| Parameter | Value | How |
|-----------|-------|-----|
| PPR (motor shaft) | 11 | RMCS-3070 datasheet |
| Gear ratio | 51.45:1 | RMCS-3070 datasheet |
| Ticks per wheel rev | 566 | 11 × 51.45 (single channel) |
| Wheel diameter | 8.5 mm | Measured |
| Wheel circumference | 26.70 mm | π × 8.5 |
| mm per tick | 0.04717 mm | 26.70 / 566 |
| Max speed | 44.5 mm/s | 100 RPM × 26.70mm / 60s |
| Wheelbase | 200 mm | Measured |

---

## Motor PI Controller

```
Per side (LEFT / RIGHT), runs at 100 Hz:

target   = cmd_vel_target × safety_scale
measured = average encoder velocity of both motors on that side
error    = target - measured
integral += error × dt          (anti-windup: ±22.25 mm/s)
output   = target + KP×error + KI×integral

KP = 0.5   KI = 0.1
```

Feed-forward term ensures the motor tracks the target even on flat ground,
while PI corrects for terrain-induced speed changes.

---

## Phase 2 Roadmap — MCP23017

When the CJMCU-2317 MCP23017 arrives:

```
Connect to CN2:
  VCC → Pin 5,  GND → Pin 7
  SDA → Pin 1,  SCL → Pin 3
  A0,A1,A2 → GND  (I2C address 0x20)
  INTA → D5 (PI0),  INTB → D7 (PI3)

GPA0 → FL encoder CH_B
GPA1 → FR encoder CH_B
GPA2 → RL encoder CH_B
GPA3 → RR encoder CH_B

Benefit:
  Full quadrature counting (direction from encoder itself)
  Accurate reverse odometry
  Better SLAM map quality
  Slip detection in both directions
```

---

## References

1. **Mourougane R., Dhanalakshmi S.** (2024). *"Terrain-Aware Velocity
   Regulation for Mobile Robots via Lightweight Execution-Level Safety
   Layer."* SRM Institute of Science and Technology.

2. **Madgwick S.O.H., Harrison A.J.L., Vaidyanathan R.** (2011).
   *"Estimation of IMU and MARG orientation using a gradient descent
   algorithm."* IEEE ICORR. doi:10.1109/ICORR.2011.5975346

3. **InvenSense** (2019). *ICM-20948 Product Specification Rev 1.3.*

4. **STMicroelectronics** (2025). *UM1907 Rev 6 — STM32F746G-DISCO
   User Manual.*
