# STM32F746G-DISCO — Terrain Bot Firmware

Real-time sensor hub for the terrain-aware differential drive robot.
Runs on the **STM32F746G-DISCO** (Cortex-M7 @ 216 MHz, FPU, 1MB Flash).

---

## What this board does

```
┌─────────────────────────────────────────────────────────────────┐
│              STM32F746G-DISCO (primary RT controller)           │
│                                                                 │
│  I2C1 (PB8/PB9)     ┌──────────────┐                          │
│  ────────────────►  │  ICM-20948   │  9-DOF IMU               │
│                      │  (accel+gyro │  AK09916 mag             │
│                      │   +mag)      │  internal I2C master     │
│                      └──────────────┘                          │
│                                                                 │
│  TIM5 (PA0=CH1, PA1=CH2)  ──► LEFT encoder (RMCS-3070)        │
│  TIM2 (PA15=CH1, PB3=CH2) ──► RIGHT encoder (RMCS-3070)       │
│                                                                 │
│  Madgwick AHRS (β=0.033) ◄── fuses accel+gyro+mag @ 100 Hz   │
│                                                                 │
│  USART6 (PC6=TX, PC7=RX)  ─────────────────────────────────►  │
│  2 Mbaud, micro-ROS           USB-to-TTL RS232 cable           │
└─────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼ USB-TTL
┌─────────────────────────────────────────────────────────────────┐
│         Raspberry Pi 4B  (ROS2 Humble)                         │
│  micro-ROS agent ◄─► /imu/data + /wheel_ticks + /wheel_velocity│
└─────────────────────────────────────────────────────────────────┘
```

---

## Hardware pin connections

All labels are as printed on the **STM32F746G-DISCO Arduino header**.

### ICM-20948 (9-DOF IMU)

| ICM-20948 pin | STM32 Arduino label | STM32 internal | Function       |
|---------------|---------------------|----------------|----------------|
| VCC           | 3.3V                | —              | Power          |
| GND           | GND                 | —              | Ground         |
| SCL           | SCL (Arduino SCL)   | PB8            | I2C1 clock     |
| SDA           | SDA (Arduino SDA)   | PB9            | I2C1 data      |
| AD0           | GND                 | —              | I2C addr = 0x68|

### USB-to-TTL RS232 cable (CP2102 / CH340)

| RS232 cable   | STM32 Arduino label | STM32 internal | Note                |
|---------------|---------------------|----------------|---------------------|
| RXD (white)   | D1                  | PC6            | USART6 TX (STM→RPi) |
| TXD (green)   | D0                  | PC7            | USART6 RX (RPi→STM) |
| GND (black)   | GND                 | —              | Common ground       |
| VCC (red)     | —                   | —              | Do NOT connect (RPi powered separately) |

### LEFT encoder (RMCS-3070, motor 1)

| Encoder pin | STM32 Arduino label | STM32 internal | Timer channel |
|-------------|---------------------|----------------|---------------|
| A (channel 1)| A0                 | PA0            | TIM5 CH1      |
| B (channel 2)| A1                 | PA1            | TIM5 CH2      |
| VCC          | 5V                 | —              | Power         |
| GND          | GND                | —              | Ground        |

### RIGHT encoder (RMCS-3070, motor 2)

| Encoder pin | STM32 Arduino label | STM32 internal | Timer channel |
|-------------|---------------------|----------------|---------------|
| A (channel 1)| D9                 | PA15           | TIM2 CH1      |
| B (channel 2)| CN2 pin 15         | PB3            | TIM2 CH2      |
| VCC          | 5V                 | —              | Power         |
| GND          | GND                | —              | Ground        |

> **Note:** D0 and D1 are used by USART6 (micro-ROS). D2–D8 are available for
> future motor control PWM. All TIM3/TIM8 channels conflict with USART6 on
> the Arduino header, leaving TIM5 and TIM2 as the only clean encoder timers.

---

## CubeMX peripheral configuration

Open the `.ioc` file and configure:

| Peripheral | Mode              | Settings                          |
|------------|-------------------|-----------------------------------|
| I2C1       | I2C               | Speed = 400 kHz (Fast Mode)       |
| USART6     | Asynchronous      | Baud = 2000000, 8N1, no flow ctrl |
| TIM5       | Encoder Mode      | CH1+CH2, TI1FP1+TI2FP2, ×4 mode  |
| TIM2       | Encoder Mode      | CH1+CH2, TI1FP1+TI2FP2, ×4 mode  |
| FPU        | Enabled           | Single-precision, default         |

---

## Files

```
Core/
├── Inc/
│   ├── app.h                    Application entry point
│   ├── icm20948.h               ICM-20948 driver (I2C, AK09916 mag)
│   ├── madgwick.h               Madgwick AHRS filter (IEEE ICORR 2011)
│   ├── encoder.h                Quadrature encoder driver (TIM5/TIM2)
│   └── microros_transport.h     USART6 micro-ROS custom transport
└── Src/
    ├── app.c                    Main application loop
    ├── icm20948.c               ICM-20948 driver implementation
    ├── madgwick.c               Madgwick AHRS implementation
    ├── encoder.c                Encoder driver implementation
    └── microros_transport.c     micro-ROS UART transport
PIN_CONNECTIONS.txt              Full wiring table (Arduino labels)
```

---

## Madgwick AHRS filter

### Why Madgwick instead of a complementary filter?

| Property           | Complementary filter | Madgwick AHRS         |
|--------------------|----------------------|-----------------------|
| Representation     | Euler angles         | Unit quaternion       |
| Gimbal lock        | Yes (±90° pitch)     | No                    |
| Yaw observable     | No (drifts)          | Yes (with mag)        |
| Sensor modes       | IMU only             | IMU + MARG (9-DOF)    |
| Tuning parameters  | 1 (α)                | 1 (β)                 |
| Computation        | O(1), simple         | O(1), ~40 mults/step  |

### Algorithm (Madgwick et al., IEEE ICORR 2011)

The filter minimises the objective function:

```
f(q, ŝ_E, s_S) = q* ⊗ ŝ_E ⊗ q - s_S
```

where `q` is the orientation quaternion, `ŝ_E` is the expected sensor
direction in Earth frame, and `s_S` is the normalised sensor reading.

Gradient descent step (eq. 25 / 32):
```
∇f = J^T · f
q̇_ε = -β · ∇f / |∇f|
```

Full update (eq. 42):
```
q̇ = q̇_ω + q̇_ε
q_{t+1} = q_t + q̇ · Δt
q_{t+1} = q_{t+1} / |q_{t+1}|   (renormalise)
```

### β parameter derivation

```
β = √(3/4) × ω_err

ω_err = gyro_noise_density × √ODR × (π/180)
      = 0.015 dps/√Hz × √100 Hz × π/180
      = 0.015 × 10 × 0.01745
      = 0.00262 rad/s

β ≈ 0.00262 × 0.866 ≈ 0.0023   (theoretical)
β = 0.033                       (conservative, per Madgwick 2011 §IV-C)
```

Higher β → faster response to accel/mag, more noise in orientation.
Lower β → smoother orientation, slower to correct gyro drift.

### MARG vs IMU-only selection

```c
if (imu.mx != 0.0f || imu.my != 0.0f || imu.mz != 0.0f)
    Madgwick_UpdateMARG(...)   // 9-DOF: yaw observable
else
    Madgwick_UpdateIMU(...)    // 6-DOF: yaw drifts
```

The AK09916 magnetometer (inside ICM-20948) is read via auxiliary I2C
at 100 Hz with no extra GPIO. When `mx=my=mz=0`, either the sensor is
demagnetised or the I2C read failed — fall back to 6-DOF.

---

## micro-ROS topics published

| Topic              | Type                         | Rate   | Description                       |
|--------------------|------------------------------|--------|-----------------------------------|
| `/imu/data`        | `sensor_msgs/Imu`            | 100 Hz | Quaternion + angular_vel + accel  |
| `/wheel_ticks`     | `std_msgs/Int32MultiArray`   | 50 Hz  | `[LEFT_ticks, RIGHT_ticks]`       |
| `/wheel_velocity`  | `std_msgs/Float32MultiArray` | 50 Hz  | `[LEFT_mm/s, RIGHT_mm/s]`         |

### IMU message covariance matrices

Populated from ICM-20948 datasheet noise specs:

```
Gyro noise:     0.015 dps/√Hz @ 100 Hz → σ = 0.00262 rad/s
Accel noise:    230 µg/√Hz   @ 100 Hz → σ = 0.02256 m/s²
Orientation:    Madgwick convergence  → σ ≈ 0.01 rad  (0.0001 = σ²)

orientation_covariance      = diag(0.0001, 0.0001, 0.0001)
angular_velocity_covariance = diag(6.86e-6, 6.86e-6, 6.86e-6)
linear_accel_covariance     = diag(5.09e-4, 5.09e-4, 5.09e-4)
```

These covariances flow into the Raspberry Pi EKF (robot_localization)
for optimal sensor fusion weighting.

---

## Encoder specifications (RMCS-3070)

| Parameter          | Value            | Derivation                        |
|--------------------|------------------|-----------------------------------|
| Motor speed        | 100 RPM (no load)| datasheet                        |
| Wheel diameter     | 8.5 mm           | measured                         |
| Wheel circumference| 26.70 mm         | π × 8.5                          |
| Encoder resolution | 566 CPR          | datasheet                        |
| Ticks/rev (×4)     | 2264             | 566 × 4 (quadrature)             |
| mm per tick        | 0.01179 mm       | 26.70 / 2264                     |
| Max wheel speed    | 44.5 mm/s        | 100/60 rev/s × 26.70 mm          |
| Max tick rate      | 3773 ticks/s     | 2264 × 100/60                    |

TIM5 and TIM2 are configured in encoder mode (×4 counting). The 32-bit
counter wraps at ±2³¹ — no overflow handling needed for continuous odometry
over typical mission durations.

---

## Integration steps

1. Open CubeMX, generate code for STM32F746G-DISCO
2. Configure I2C1, USART6, TIM2, TIM5 as shown above
3. Copy all files from `Core/Inc/` and `Core/Src/` into the generated project
4. Add to the bottom of `main()`:
   ```c
   App_Run();   /* does not return */
   ```
5. Ensure `microros` library is linked (follow micro-ROS STM32 build guide)
6. Build and flash with STM32CubeIDE

---

## References

1. **Madgwick, S.O.H., Harrison, A.J.L., Vaidyanathan, R.** (2011).
   "Estimation of IMU and MARG orientation using a gradient descent algorithm."
   *Proc. IEEE Int. Conf. Rehabilitation Robotics (ICORR)*, pp. 1-7.
   doi:[10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346)

2. **Mahony, R., Hamel, T., Pflimlin, J.M.** (2008).
   "Nonlinear complementary filters on the special orthogonal group."
   *IEEE Trans. Autom. Control*, 53(5), pp. 1203-1218.
   doi:[10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738)

3. **Abbate, A., et al.** (2012).
   "A smartphone-based fall detection system."
   *Pervasive and Mobile Computing*, 8(6), pp. 883-899.
   (ICM-20948 bias calibration technique)

4. **Invensense** (2019). *ICM-20948 Product Specification Rev 1.3.*
   TDK InvenSense, San Jose, CA.
