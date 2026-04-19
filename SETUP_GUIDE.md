# STM32F746G-DISCO Terrain Bot — Complete Setup Guide

---

## 1. Hardware You Need

| Component | Qty | Notes |
|-----------|-----|-------|
| STM32F746G-DISCO | 1 | Main controller |
| Raspberry Pi 4B | 1 | Runs ROS2 Humble + Nav2 |
| ICM-20948 IMU module | 1 | 9-DOF, I2C |
| PCA9685 PWM module | 1 | I2C motor PWM expander |
| RMCS-3070 motors | 4 | 100 RPM, 12V with encoders |
| H-bridge driver | 4 | L298N or TB6612FNG, one per motor |
| USB-to-TTL cable | 1 | CP2102 or CH340, 3.3V logic |
| 12V power supply | 1 | For motors |
| 5V power supply | 1 | For STM32 + RPi |
| Jumper wires | — | Male-to-female |

---

## 2. Complete Wiring Diagram

### ICM-20948 IMU → STM32
```
ICM-20948        STM32 label     STM32 pin
─────────        ───────────     ─────────
VDD (3.3V)  →   3V3             —
GND         →   GND             —
SCL         →   SCL (Arduino)   PB8
SDA         →   SDA (Arduino)   PB9
AD0         →   GND             — (sets I2C addr = 0x68)
INT         →   D2  (optional)  PG6
```

### PCA9685 → STM32  (shares I2C1 with ICM-20948)
```
PCA9685     STM32 label     STM32 pin
───────     ───────────     ─────────
VCC    →    3V3             —
GND    →    GND             —
SCL    →    SCL (Arduino)   PB8
SDA    →    SDA (Arduino)   PB9
OE     →    GND             — (always enabled)
A0-A5  →    GND             — (I2C addr = 0x40)
```
> Both ICM-20948 and PCA9685 share the same SCL/SDA lines. This is fine
> because they have different I2C addresses (0x68 vs 0x40).

### PCA9685 → H-bridges → Motors
```
PCA9685 CH   →   H-bridge pin   Motor
────────────     ─────────────  ─────
CH0 (PWM)   →   ENA            Front-Left  (FL)
CH4 (DIR)   →   IN1            Front-Left  (FL)
GND         →   IN2            Front-Left  (keep LOW)

CH1 (PWM)   →   ENA            Front-Right (FR)
CH5 (DIR)   →   IN1            Front-Right (FR)
GND         →   IN2            Front-Right

CH2 (PWM)   →   ENA            Rear-Left   (RL)
CH6 (DIR)   →   IN1            Rear-Left   (RL)
GND         →   IN2            Rear-Left

CH3 (PWM)   →   ENA            Rear-Right  (RR)
CH7 (DIR)   →   IN1            Rear-Right  (RR)
GND         →   IN2            Rear-Right
```
> H-bridge motor supply (VM/VMOT) → 12V
> H-bridge logic supply (VCC/VDD) → 5V

### Encoders → STM32 Timers
```
Motor         Timer  CH_A pin          CH_B pin
───────────── ─────  ────────────────  ──────────────────────
Front-Left    TIM5   A0 (PA0)          A1 (PA1)
Front-Right   TIM2   D9 (PA15)         Morpho CN2 pin15 (PB3)
Rear-Left     TIM4   Morpho CN2 pin13  Morpho CN2 pin11
                     (PB6)             (PB7)
Rear-Right    TIM3   D3 (PB4)          Morpho CN1 pin64 (PB5)
```
RMCS-3070 encoder cable colours:
```
Red   → 5V
Black → GND
Green → CH_A (timer CH1)
White → CH_B (timer CH2)
```

### micro-ROS UART → Raspberry Pi
```
STM32 label   STM32 pin   USB-TTL cable wire   RPi
───────────   ─────────   ──────────────────   ───
D1 (TX)  →   PC6         White (RXD)          USB end
D0 (RX)  →   PC7         Green (TXD)          USB end
GND      →   GND         Black (GND)          USB end
(Red VCC wire → DO NOT connect)
```

---

## 3. STM32CubeMX Configuration

Open your .ioc file and configure the following:

### System Core
```
SYS → Debug: Serial Wire (SWD)
RCC → HSE: CRYSTAL/CERAMIC RESONATOR
Clock Configuration:
  - Input: HSE 25 MHz (ST-LINK provides this via BYPASS mode)
  - PLL: M=25, N=432, P=2, Q=9
  - SYSCLK = 216 MHz
```

### Peripherals to Enable

| Peripheral | Mode | Key Settings |
|------------|------|-------------|
| I2C1 | I2C | Fast Mode 400 kHz, SCL=PB8, SDA=PB9 |
| USART6 | Asynchronous | 2000000 baud, 8N1, TX=PC6, RX=PC7 |
| TIM5 | Encoder Mode | CH1=PA0, CH2=PA1, Period=65535, Filter=4 |
| TIM2 | Encoder Mode | CH1=PA15, CH2=PB3, Period=65535, Filter=4 |
| TIM4 | Encoder Mode | CH1=PB6, CH2=PB7, Period=65535, Filter=4 |
| TIM3 | Encoder Mode | CH1=PB4, CH2=PB5, Period=65535, Filter=4 |
| LTDC | Built-in LCD | Leave as default BSP config |

> **TIM1 is NOT needed** — motor PWM goes through PCA9685 over I2C.

All encoder timer settings:
```
Combined Channels : Encoder Mode TI1 and TI2
Counter Period    : 65535
Prescaler         : 0
CH1/CH2 Polarity  : Rising Edge
Input Filter      : 4
```

### GPIO Outputs (not needed anymore!)
```
PG6, PG7, PI2, PI3 were previously motor direction pins.
These are now handled by PCA9685 CH4-CH7.
You can leave them as-is or remove them from CubeMX.
```

---

## 4. CubeIDE Project Setup

### Step-by-step:
1. Generate code from CubeMX → Open in STM32CubeIDE
2. Copy all files from `Core/Inc/` and `Core/Src/` into your project
3. In `Core/Src/main.c`, add at the bottom of `main()`:
   ```c
   /* USER CODE BEGIN 2 */
   App_Run();   /* never returns */
   /* USER CODE END 2 */
   ```
4. Add micro-ROS static library to your project (see Step 5)
5. Build → Run

### micro-ROS library setup:
```bash
# On your PC, clone the micro-ROS component for STM32
git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git

# Follow the README to generate the static library for STM32F7
# Copy libmicroros.a and include/ into your CubeIDE project
```

---

## 5. Raspberry Pi Setup

### Install ROS2 Humble
```bash
# On RPi (Ubuntu 22.04)
sudo apt update && sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install micro-ROS agent
```bash
sudo apt install -y ros-humble-micro-ros-agent
```

### Find the STM32 serial device
```bash
# Plug in the USB-TTL cable, then:
ls /dev/ttyUSB*    # usually /dev/ttyUSB0
# or
dmesg | tail -20   # look for CP210x or CH340
```

### Create a udev rule (so device name is always the same)
```bash
# Get the idVendor and idProduct
udevadm info -a /dev/ttyUSB0 | grep idVendor
udevadm info -a /dev/ttyUSB0 | grep idProduct

# Create rule (example for CP2102):
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="stm32"' \
  | sudo tee /etc/udev/rules.d/99-stm32.rules
sudo udevadm control --reload-rules
# Now device is always /dev/stm32
```

### Start micro-ROS agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm32 -b 2000000
# or without udev rule:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000
```

---

## 6. First Boot Sequence

1. Connect all hardware per wiring diagram
2. Power on RPi → start micro-ROS agent (Step 5)
3. Power on STM32 via USB
4. LCD shows **[CALIB..]** → keep robot completely still for ~2 seconds
5. LCD shows **[RUN]** → robot is live

### Verify topics are publishing:
```bash
ros2 topic list
# Expected:
# /terrain_bot/imu/data
# /terrain_bot/wheel_ticks
# /terrain_bot/wheel_velocity

ros2 topic hz /terrain_bot/imu/data        # ~100 Hz
ros2 topic hz /terrain_bot/wheel_velocity  # ~50 Hz
```

---

## 7. Motor Testing

### Manual velocity commands:
```bash
# Drive forward at 30 mm/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.030}, angular: {z: 0.0}}" --once

# Turn left in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### If a motor spins backwards:
Edit `Core/Src/motor.c`, change the polarity for that motor:
```c
static const int8_t MOTOR_POLARITY[MOTOR_NUM] = {
    +1,   /* FL */
    -1,   /* FR */   // ← change to +1 if FR spins wrong
    +1,   /* RL */
    -1,   /* RR */
};
```

---

## 8. Safety Layer Tuning

All thresholds are in `Core/Src/app.c`:

```c
#define SAFETY_THETA_SAFE    10.0f   // Below this angle → full speed
#define SAFETY_THETA_CRIT    30.0f   // Above this → E-stop
#define SAFETY_ESTOP_PITCH   30.0f   // Hard stop pitch threshold
#define SAFETY_ESTOP_ROLL    40.0f   // Hard stop roll threshold
#define SAFETY_LPF_BETA      0.90f   // Smoothing (higher = smoother)
```

Increase `SAFETY_ESTOP_PITCH` if robot stops too early on gentle ramps.
Decrease `SAFETY_LPF_BETA` if you want faster response to terrain changes.

---

## 9. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| LCD stuck on CALIB | micro-ROS agent not running | Start agent on RPi first |
| No topics on RPi | UART wiring wrong | Swap D0/D1 wires |
| Motors don't move | PCA9685 not responding | Check I2C wiring, confirm A0-A5 = GND |
| One motor wrong direction | Mirror-mount polarity | Change MOTOR_POLARITY in motor.c |
| Encoder counts don't change | Timer not in encoder mode | Recheck CubeMX TIM config |
| IMU init fails | I2C address conflict or wiring | Check AD0=GND on ICM-20948 |
| Robot stops immediately | Safety E-stop triggering | Place robot flat, re-flash |
| Speed always low | Terrain risk high on flat surface | Re-run calibration on flat surface |

---

## 10. SLAM with Nav2

Once manual control works:
```bash
# Terminal 1 — micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm32 -b 2000000

# Terminal 2 — SLAM
ros2 launch slam_toolbox online_async_launch.py

# Terminal 3 — Nav2
ros2 launch nav2_bringup navigation_launch.py

# Terminal 4 — RViz
ros2 run rviz2 rviz2
```

Nav2 publishes `/cmd_vel` automatically. The STM32 safety layer will
reduce speed on slopes and E-stop if pitch > 30° — all transparent to Nav2.
