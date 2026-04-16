# STM32F746G-DISCO — Terrain Bot Firmware

Real-time motor control, IMU sensing, and ROS2 bridge for a 4-wheel differential-drive terrain robot.
Runs on the **STM32F746G-DISCO** (Cortex-M7 @ 216 MHz, hardware FPU, 1 MB Flash).

---

## System architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                 STM32F746G-DISCO  (real-time controller)             │
│                                                                      │
│  I2C1 (PB8/PB9)  ──────────►  ICM-20948 (9-DOF IMU)               │
│                                  accel + gyro + AK09916 mag         │
│                                  Madgwick AHRS filter @ 100 Hz      │
│                                                                      │
│  TIM5 (PA0/PA1)      ──────►  Encoder FL  (Front-Left)             │
│  TIM2 (PA15/PB3)     ──────►  Encoder FR  (Front-Right)            │
│  TIM4 (PB6/PB7)      ──────►  Encoder RL  (Rear-Left)              │
│  TIM3 (PB4/PB5)      ──────►  Encoder RR  (Rear-Right)             │
│                                                                      │
│  TIM1_CH1 (PA8)  + DIR (PG6) ──►  Motor FL                        │
│  TIM1_CH2 (PE11) + DIR (PG7) ──►  Motor FR                        │
│  TIM1_CH3 (PE13) + DIR (PI3) ──►  Motor RL                        │
│  TIM1_CH4 (PE14) + DIR (PI2) ──►  Motor RR                        │
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  TERRAIN-AWARE SAFETY LAYER  (100 Hz, paper §III)           │    │
│  │  Weighted inclination  →  risk normalization  →  LPF(β=0.90)│    │
│  │  speed_scale = (1 − R_filtered)²       [paper Eq. 11]       │    │
│  │  E-stop:  pitch > 30° or roll > 40°    [paper Eq. 12]       │    │
│  │  Flip detect: pitch_rate > 30°/s + pitch > 15°              │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
│  USART6 (PC6=TX, PC7=RX)  ◄──────────────────────────────────────  │
│  2 Mbaud, micro-ROS custom transport                                 │
└──────────────────────────────────────────────────────────────────────┘
                            │ USB-to-TTL (CP2102 / CH340)
                            ▼
┌──────────────────────────────────────────────────────────────────────┐
│             Raspberry Pi 4B  (ROS2 Humble)                          │
│                                                                      │
│  micro-ROS agent  ◄──►  STM32 node "terrain_bot/stm32f7_node"      │
│                                                                      │
│  Subscribes  /imu/data         sensor_msgs/Imu            100 Hz    │
│              /wheel_ticks      std_msgs/Int32MultiArray    50 Hz     │
│              /wheel_velocity   std_msgs/Float32MultiArray  50 Hz     │
│                                                                      │
│  Publishes   /cmd_vel          geometry_msgs/Twist  (Nav2 / SLAM)   │
│              linear.x [m/s] + angular.z [rad/s]                     │
└──────────────────────────────────────────────────────────────────────┘
```

---

## File structure

```
Core/
├── Inc/
│   ├── app.h                   Application entry point declaration
│   ├── icm20948.h              ICM-20948 9-DOF IMU driver API
│   ├── madgwick.h              Madgwick AHRS filter (IEEE ICORR 2011)
│   ├── encoder.h               4-wheel quadrature encoder driver
│   ├── motor.h                 4-motor PWM velocity driver (TIM1)
│   ├── lcd_display.h           Built-in 480×272 LCD dashboard
│   └── microros_transport.h    USART6 micro-ROS transport
└── Src/
    ├── app.c                   Main loop, safety layer, motor PID, ROS bridge
    ├── icm20948.c              ICM-20948 I2C driver + AK09916 magnetometer
    ├── madgwick.c              Madgwick MARG/IMU filter (quaternion)
    ├── encoder.c               Encoder read via TIM5/TIM2/TIM4/TIM3
    ├── motor.c                 TIM1 PWM + GPIO direction control
    ├── lcd_display.c           LTDC LCD rendering (BSP)
    └── microros_transport.c    UART send/receive callbacks for micro-ROS
PIN_CONNECTIONS.txt             Full wiring table with Arduino and Morpho labels
```

---

## Hardware connections

### 1. ICM-20948 (9-DOF IMU)

| ICM-20948 pin | Board label | STM32 pin | CubeMX function |
|---------------|-------------|-----------|-----------------|
| VCC           | 3V3         | —         | Power           |
| GND           | GND         | —         | Ground          |
| SCL           | SCL         | PB8       | I2C1_SCL        |
| SDA           | SDA         | PB9       | I2C1_SDA        |
| AD0           | GND         | —         | I2C addr = 0x68 |
| INT           | D2 (opt.)   | PG6       | EXTI (optional) |

> The SCL/SDA labels are on the left side of the Arduino header, separate from D0-D13.

---

### 2. micro-ROS UART (STM32 ↔ Raspberry Pi)

| Cable wire    | Board label | STM32 pin | Direction        |
|---------------|-------------|-----------|------------------|
| RXD (white)   | D1          | PC6       | STM32 TX → RPi   |
| TXD (green)   | D0          | PC7       | RPi TX → STM32   |
| GND (black)   | GND         | —         | Common ground    |
| VCC (red)     | —           | —         | **Do NOT connect** |

Baud rate: **2 000 000** (2 Mbaud), 8N1, no flow control.

On the Raspberry Pi:
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000
```

---

### 3. Encoders (4× RMCS-3070)

| Motor        | Timer | CH_A label        | CH_A pin | CH_B label         | CH_B pin |
|--------------|-------|-------------------|----------|--------------------|----------|
| Front-Left   | TIM5  | A0 (Arduino)      | PA0      | A1 (Arduino)       | PA1      |
| Front-Right  | TIM2  | D9 (Arduino)      | PA15     | Morpho CN2 pin 15  | PB3      |
| Rear-Left    | TIM4  | Morpho CN2 pin 13 | PB6      | Morpho CN2 pin 11  | PB7      |
| Rear-Right   | TIM3  | D3 (Arduino)      | PB4      | Morpho CN2         | PB5      |

Encoder cable colour code (RMCS-3070):

| Wire colour | Connect to              |
|-------------|-------------------------|
| Red         | 5V (encoder VCC)        |
| Black       | GND                     |
| Green       | CH_A (timer CH1 pin)    |
| White       | CH_B (timer CH2 pin)    |

---

### 4. Motor PWM + direction (4× H-bridge)

One H-bridge driver per motor (e.g. L298N, TB6612FNG, DRV8833):

| Motor        | PWM pin         | STM32 pin | Direction pin | STM32 pin |
|--------------|-----------------|-----------|---------------|-----------|
| Front-Left   | TIM1_CH1        | PA8 (D10) | PG6 (D2)      | DIR GPIO  |
| Front-Right  | TIM1_CH2 (Morpho)| PE11     | PG7 (D4)      | DIR GPIO  |
| Rear-Left    | TIM1_CH3 (Morpho)| PE13     | PI3 (D7)      | DIR GPIO  |
| Rear-Right   | TIM1_CH4 (Morpho)| PE14     | PI2 (D8)      | DIR GPIO  |

H-bridge wiring per motor:
```
STM32 PWM pin  →  ENA  (speed)
STM32 DIR pin  →  IN1  (direction)
GND            →  IN2  (keep LOW for single DIR logic)
Motor terminals → motor A / B
```

> **Right-side motors** (FR, RR) are physically mirror-mounted. The firmware
> handles this automatically via `MOTOR_POLARITY[]` in `motor.c`.
> If a wheel spins backwards, change its polarity from `-1` to `+1`.

---

## CubeMX configuration

Open CubeMX (or the `.ioc` file) and configure:

### Existing peripherals (unchanged)

| Peripheral | Mode         | Settings                                    |
|------------|--------------|---------------------------------------------|
| I2C1       | I2C          | Fast Mode, 400 kHz; SDA=PB9, SCL=PB8       |
| USART6     | Asynchronous | 2 000 000 baud, 8N1, no flow; TX=PC6, RX=PC7|
| TIM5       | Encoder      | CH1=PA0, CH2=PA1, Period=65535, Filter=4    |
| TIM2       | Encoder      | CH1=PA15, CH2=PB3, Period=65535, Filter=4   |
| LTDC       | Built-in LCD | Default BSP config (480×272)                |

### New peripherals (add these)

| Peripheral | Mode         | Settings                                              |
|------------|--------------|-------------------------------------------------------|
| TIM4       | Encoder      | CH1=PB6, CH2=PB7, Period=65535, Filter=4              |
| TIM3       | Encoder      | CH1=PB4, CH2=PB5, Period=65535, Filter=4              |
| TIM1       | PWM (4-ch)   | PSC=0, ARR=10799 → 20 kHz; CH1=PA8, CH2=PE11, CH3=PE13, CH4=PE14 |
| PG6        | GPIO_Output  | Motor FL direction (Label: `MOTOR_FL_DIR`)            |
| PG7        | GPIO_Output  | Motor FR direction (Label: `MOTOR_FR_DIR`)            |
| PI3        | GPIO_Output  | Motor RL direction (Label: `MOTOR_RL_DIR`)            |
| PI2        | GPIO_Output  | Motor RR direction (Label: `MOTOR_RR_DIR`)            |

TIM1 PWM settings (applies to all 4 channels):
```
Prescaler (PSC)     : 0
Counter Period (ARR): 10799       → 216 MHz / 10800 = 20 kHz
Auto-reload preload : Enable
PWM Mode            : Mode 1
Polarity            : High
```

All encoder timer settings:
```
Combined Channels   : Encoder Mode TI1 and TI2
Counter Period      : 65535
Prescaler           : 0
CH1/CH2 Polarity    : Rising Edge
Input Filter        : 4
```

---

## Integration into a CubeMX project

```
1. Generate CubeMX project for STM32F746G-DISCO
2. Configure all peripherals listed above
3. Copy these files into the generated project:
      Core/Inc/  →  app.h  icm20948.h  madgwick.h  encoder.h
                     motor.h  lcd_display.h  microros_transport.h
      Core/Src/  →  app.c  icm20948.c  madgwick.c  encoder.c
                     motor.c  lcd_display.c  microros_transport.c
4. In Core/Src/main.c, add at the bottom of main() before the while(1):
      App_Run();   /* does not return */
5. Link the micro-ROS static library (follow micro-ROS STM32 build guide)
6. Build and flash (STM32CubeIDE or STM32CubeProgrammer)
```

---

## Flashing to the STM32F746G-DISCO

### Method 1 — STM32CubeIDE (recommended)

```
1. File → Import → Existing project into workspace
2. Build Project  (Ctrl+B)
3. Connect the board via ST-LINK USB (the USB connector labeled "ST-LINK")
4. Run → Debug (F11)  or  Run → Run (Ctrl+F11)
```

### Method 2 — STM32CubeProgrammer (CLI)

```bash
# Windows PowerShell
STM32_Programmer_CLI.exe -c port=SWD -w build/terrain_bot.elf -v -rst
```

### Method 3 — Drag-and-drop (mass storage mode)

```
1. Press and hold the blue USER button, then press RESET
2. Board appears as USB mass storage "DIS_F746NG"
3. Drag and drop the .bin file from build/ onto the drive
4. Press RESET to boot the new firmware
```

---

## ROS2 side (Raspberry Pi 4B)

### 1. Start micro-ROS agent

```bash
# Install micro-ROS agent if not already done
pip install micro-ros-agent   # or build from source

# Start agent (replace ttyUSB0 with your device)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000
```

### 2. Verify STM32 topics are live

```bash
ros2 topic list
# Should show:
# /terrain_bot/imu/data
# /terrain_bot/wheel_ticks
# /terrain_bot/wheel_velocity

ros2 topic hz /terrain_bot/imu/data        # should read ~100 Hz
ros2 topic hz /terrain_bot/wheel_velocity  # should read ~50 Hz
```

### 3. Send velocity commands (manual test)

```bash
# Drive forward at 30 mm/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.030}, angular: {z: 0.0}}" --once

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

> **cmd_vel watchdog:** If no `/cmd_vel` is received for 500 ms, the STM32
> automatically stops all motors and resets the velocity integrators.
> This protects the robot if ROS2 or the network goes down.

### 4. SLAM integration (Nav2)

The STM32 is a drop-in actuator for any ROS2 nav stack. Typical setup:

```bash
# Example: run slam_toolbox + Nav2
ros2 launch slam_toolbox online_async_launch.py
ros2 launch nav2_bringup navigation_launch.py
```

Nav2 publishes `/cmd_vel` automatically. The STM32 converts it to wheel
velocities using differential-drive kinematics and runs a closed-loop
velocity PI controller.

---

## Terrain-aware safety layer

Based on:
> Mourougane R., Dhanalakshmi S. *"Terrain-Aware Velocity Regulation for Mobile
> Robots via Lightweight Execution-Level Safety Layer."*
> SRM Institute of Science and Technology, Kattankulatham.

The safety layer runs **entirely on the STM32 at 100 Hz**, making immediate
hardware-level decisions without waiting for ROS.

### Formula chain (paper §III)

```
Step 1 — Inclination  (Eq. 5)
  θ_w = 0.7 × |pitch_deg| + 0.3 × |roll_deg|
  (pitch weighted higher = slope risk; roll = lateral instability)

Step 2 — Normalize to [0, 1]  (Eq. 9)
  R_incl = clamp((θ_w − 10°) / (30° − 10°), 0, 1)
  (0 in safe zone <10°, ramps to 1 at critical threshold 30°)

Step 3 — Terrain shake  (Eq. 6-7)
  R_shake = |‖a‖ − g_calib| / g_calib
  (captures bumps and surface vibrations)

Step 4 — Combined risk  (Eq. 8)
  R_raw = 0.7 × R_incl + 0.3 × R_shake

Step 5 — Low-pass filter  β = 0.90  (Eq. 10)
  R_f = 0.90 × R_f_prev + 0.10 × R_raw
  (smooths noise spikes from uneven terrain)

Step 6 — Nonlinear velocity scale  (Eq. 11)
  speed_scale = (1 − R_f)²
  (quadratic: gentle at low risk, exponential deceleration at high risk)

Step 7 — Hard E-stop  (Eq. 12)
  if pitch > 30° OR roll > 40°  →  speed_scale = 0, motors stop immediately

Step 8 — Forward-flip detection  (paper §VI)
  if |pitch_rate| > 30°/s AND pitch > 15°  →  E-stop
```

### Speed zones (from paper Fig. 5)

| Zone    | Pitch range | speed_scale  | Behaviour                     |
|---------|-------------|--------------|-------------------------------|
| Safe    | 0 – 10°     | 1.00         | Full commanded speed          |
| Caution | 10 – 30°    | (1−R)² curve | Progressive deceleration      |
| E-stop  | > 30°       | 0.00         | Immediate motor stop          |

Validated across 10 000+ data points in Gazebo (construction, inspection, ramp worlds).
Up to **77% velocity reduction** on steep slopes.

---

## Motor velocity PI controller

Each motor has its own closed-loop velocity PI controller running at 100 Hz.

```
target  = cmd_vel_target × speed_scale      ← safety scale applied first
error   = target − encoder_velocity          ← from quadrature counter
integral += error × dt                       ← anti-windup: ±ILIMIT
cmd     = target + KP×error + KI×integral   ← feed-forward + PI correction
Motor_Set(motor, cmd)                        ← velocity → PWM duty
```

Default gains (tune for your motors):

| Parameter | Value    | Description                          |
|-----------|----------|--------------------------------------|
| KP        | 0.5      | Proportional gain                    |
| KI        | 0.1      | Integral gain                        |
| ILIMIT    | 22.25 mm/s | Anti-windup clamp (½ max speed)    |

---

## Madgwick AHRS filter

### Why Madgwick over a complementary filter?

| Property       | Complementary filter | Madgwick AHRS        |
|----------------|----------------------|----------------------|
| Representation | Euler angles         | Unit quaternion      |
| Gimbal lock    | Yes (at ±90° pitch)  | No                   |
| Yaw observable | No (drifts)          | Yes (with mag)       |
| Tuning params  | 1 (α)                | 1 (β)                |
| Modes          | IMU only             | IMU + MARG (9-DOF)   |

β = 0.033 (tuned for ICM-20948 @ 102 Hz, per Madgwick 2011 §IV-C).

MARG mode (9-DOF) is used when magnetometer data is valid.
Falls back to IMU-only (6-DOF) when `mx=my=mz=0`.

---

## Encoder specifications (RMCS-3070)

| Parameter           | Value         | Derivation                  |
|---------------------|---------------|-----------------------------|
| Output shaft speed  | 100 RPM       | datasheet (no load, 12 V)   |
| Gear ratio          | 51.45:1       | datasheet                   |
| Encoder PPR (motor) | 11            | datasheet (Hall effect)     |
| Ticks per wheel rev | 2264          | 11 × 51.45 × 4 (quadrature)|
| Wheel diameter      | 8.5 mm        | measured                    |
| Wheel circumference | 26.70 mm      | π × 8.5                     |
| mm per tick         | 0.01179 mm    | 26.70 / 2264                |
| Max wheel speed     | 44.5 mm/s     | 100/60 × 26.70              |
| Wheelbase           | 200 mm        | measured (L-to-R centres)   |

> Update `ENC_WHEEL_DIAM_MM` and `ENC_WHEELBASE_MM` in `encoder.h` if your
> wheels or chassis differ from the values above.

---

## micro-ROS topics

### Published (STM32 → ROS2)

| Topic             | Type                         | Rate   | Content                                     |
|-------------------|------------------------------|--------|---------------------------------------------|
| `/imu/data`       | `sensor_msgs/Imu`            | 100 Hz | Quaternion [w,x,y,z], angular vel, accel    |
| `/wheel_ticks`    | `std_msgs/Int32MultiArray`   | 50 Hz  | [FL, FR, RL, RR] accumulated ticks         |
| `/wheel_velocity` | `std_msgs/Float32MultiArray` | 50 Hz  | [FL, FR, RL, RR] velocity in mm/s          |

Array order: `[0]=FL, [1]=FR, [2]=RL, [3]=RR`

IMU covariance (from ICM-20948 datasheet):
```
orientation_covariance      = diag(1e-4, 1e-4, 1e-4)   rad²
angular_velocity_covariance = diag(6.86e-6, …)          (rad/s)²
linear_accel_covariance     = diag(5.09e-4, …)          (m/s²)²
```

### Subscribed (ROS2 → STM32)

| Topic      | Type                      | Content                              |
|------------|---------------------------|--------------------------------------|
| `/cmd_vel` | `geometry_msgs/Twist`     | `linear.x` [m/s], `angular.z` [rad/s]|

Differential-drive conversion:
```
v_left  = linear.x × 1000  −  angular.z × (wheelbase / 2)   [mm/s]
v_right = linear.x × 1000  +  angular.z × (wheelbase / 2)   [mm/s]
FL = RL = v_left
FR = RR = v_right
```

---

## LCD dashboard (built-in 4.3" display)

The 480×272 LTDC display shows live sensor data at 10 Hz:

```
┌──────────────────────────────────────────┐
│   STM32 TERRAIN BOT          [  RUN  ]   │
├──────────────────────────────────────────┤
│  IMU - ORIENTATION                       │
│  Roll  :  +012.34 deg                    │
│  Pitch :  -005.67 deg                    │
│  Yaw   :  +089.01 deg                    │
├──────────────────────────────────────────┤
│  WHEEL VELOCITY                          │
│  Left  :  +032.10 mm/s    (FL encoder)   │
│  Right :  +031.80 mm/s    (FR encoder)   │
├──────────────────────────────────────────┤
│  [=========>              ]  pitch bar   │
└──────────────────────────────────────────┘
```

Status indicator: `[CALIB..]` during 2-second gyro calibration, `[RUN]` after.

---

## Startup sequence

```
Power on
  │
  ├─ LCD_Display_Init()
  ├─ ICM20948_Init()          — verify WHO_AM_I = 0xEA, configure AK09916
  ├─ Encoder_Init()           — start TIM5, TIM2, TIM4, TIM3 in encoder mode
  ├─ Motor_Init()             — start TIM1 PWM, all channels at 0% duty
  ├─ Madgwick_Init()          — identity quaternion [1,0,0,0]
  ├─ uros_setup()             — wait for micro-ROS agent on Raspberry Pi
  │                             (blocks until agent responds)
  ├─ Gyro calibration (~2 s)  — robot must be completely still
  │     200 samples → bias for gx, gy, gz, and gravity magnitude
  ├─ Seed Madgwick            — initial quaternion from accel gravity vector
  │
  └─ Main loop (100 Hz)
        ├─ ICM20948_Read()    — 23-byte burst read (accel+gyro+mag)
        ├─ Madgwick_Update()  — MARG or IMU-only depending on mag validity
        ├─ Madgwick_GetEuler()— update roll/pitch/yaw in radians
        ├─ update_safety()    — paper §III: risk → speed_scale
        ├─ Encoder_Update()   — read all 4 quadrature counters
        ├─ watchdog check     — stop if no /cmd_vel for 500 ms
        ├─ motor_pid_update() — PI per motor using encoder velocity
        ├─ publish_imu()      — 100 Hz
        ├─ publish_encoders() — 50 Hz
        ├─ LCD_Display_Update()— 10 Hz
        └─ executor_spin_some()— process incoming /cmd_vel
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ICM20948_Init` fails | Wrong I2C wiring or AD0 not grounded | Check SDA/SCL, verify AD0=GND |
| Robot never leaves CALIB screen | micro-ROS agent not running | Start agent on RPi first |
| `/imu/data` stops at 0 Hz | USART6 wiring issue | Swap TX/RX wires |
| Motors don't move | TIM1 not configured or H-bridge not powered | Check CubeMX TIM1 + 12V supply |
| One motor spins backwards | Mirror-mounted motor polarity wrong | Change `MOTOR_POLARITY[i]` in `motor.c` |
| Robot tips over | E-stop thresholds too conservative | Increase `SAFETY_ESTOP_PITCH/ROLL` in `app.c` |
| Speed always reduced | Flat terrain reading high risk | Re-run gyro calib on flat surface |
| Encoder counts wrong | Filter value too low | Set Input Filter = 4 in CubeMX for all encoder timers |

---

## References

1. **Mourougane R., Dhanalakshmi S.** (2024). *"Terrain-Aware Velocity Regulation
   for Mobile Robots via Lightweight Execution-Level Safety Layer."*
   SRM Institute of Science and Technology.
   *(Safety layer Eq. 5–12, speed zones, β=0.90 LPF)*

2. **Madgwick S.O.H., Harrison A.J.L., Vaidyanathan R.** (2011).
   *"Estimation of IMU and MARG orientation using a gradient descent algorithm."*
   Proc. IEEE ICORR, pp. 1-7.
   doi:[10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346)

3. **InvenSense** (2019). *ICM-20948 Product Specification Rev 1.3.*
   TDK InvenSense, San Jose, CA.

4. **STMicroelectronics** (2015). *UM1906 — STM32F746G-DISCO User Manual.*
   [st.com/resource/en/user_manual/um1906.pdf](https://www.st.com/resource/en/user_manual/um1906.pdf)
