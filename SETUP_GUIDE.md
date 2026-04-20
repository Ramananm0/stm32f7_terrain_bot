# STM32F746G-DISCO Terrain Bot — Complete Setup Guide

---

## 1. Hardware Required

| Component | Qty | Notes |
|-----------|-----|-------|
| STM32F746G-DISCO | 1 | Main controller |
| Raspberry Pi 4B | 1 | ROS2 Humble + Nav2 |
| ICM-20948 IMU | 1 | 9-DOF, I2C |
| BTS7960 motor driver | 2 | Left side + Right side |
| RMCS-3070 motors | 4 | 100 RPM, 12V, built-in encoders |
| USB-TTL cable | 1 | CP2102/CH340, 6-pin |
| 12V power supply | 1 | For motors |
| 5V power supply | 1 | For STM32 + RPi |
| Jumper wires (female-female) | — | Dupont wires |

---

## 2. Pin Connections (Arduino Header Only)

### BTS7960 #1 — LEFT side (FL + RL motors)
```
BTS7960    Board Label    STM32 Pin
RPWM    →  A4             PF7   (TIM11_CH1 forward PWM)
LPWM    →  A5             PF6   (TIM10_CH1 reverse PWM)
R_EN    →  3.3V           tie HIGH always
L_EN    →  3.3V           tie HIGH always
VCC     →  5V
GND     →  GND
RIS     →  not connected
LIS     →  not connected
B+/B-   →  FL motor + RL motor (parallel)
```

### BTS7960 #2 — RIGHT side (FR + RR motors)
```
BTS7960    Board Label    STM32 Pin
RPWM    →  A2             PF9   (TIM14_CH1 forward PWM)
LPWM    →  A3             PF8   (TIM13_CH1 reverse PWM)
R_EN    →  3.3V           tie HIGH always
L_EN    →  3.3V           tie HIGH always
VCC     →  5V
GND     →  GND
RIS     →  not connected
LIS     →  not connected
B+/B-   →  FR motor + RR motor (parallel)
```

### Encoders (1 pin per motor — CH_A only)
```
Motor    Board Label    STM32 Pin    Encoder Wire
FL    →  A0             PA0          Green (CH_A)
FR    →  D3             PB4          Green (CH_A)
RL    →  D9             PA15         Green (CH_A)
RR    →  D2             PG6          Green (CH_A)

All encoders:
Red   → 5V
Black → GND
White → not connected (CH_B — future use)
```

### ICM-20948 IMU → CN2 connector (8-pin I2C)
```
ICM-20948    CN2 Pin    Description
VDD       →  Pin 5      +3V3
GND       →  Pin 7      GND
SDA       →  Pin 1      I2C_SDA (PB9)
SCL       →  Pin 3      I2C_SCL (PB8)
AD0       →  GND        (I2C address = 0x68)
INT       →  not connected
```

### UART to Raspberry Pi (USB-TTL cable)
```
Cable Wire    Board Label    Direction
White (RXD) → D1 (PC6)      STM32 TX → RPi
Green (TXD) → D0 (PC7)      RPi TX → STM32
Black (GND) → GND
Red         → DO NOT CONNECT
Yellow/Blue → DO NOT CONNECT
```

---

## 3. CubeMX Configuration — Step by Step

### Step 1 — System Core
```
SYS → Debug: Serial Wire (SWD)
RCC → HSE: BYPASS Clock Source (ST-LINK provides 25MHz)

Clock Configuration tab:
  Input Freq  : 25 MHz
  PLL Source  : HSE
  PLLM        : 25
  PLLN        : 432
  PLLP        : 2
  SYSCLK      : 216 MHz ← confirm this
  AHB         : /1  → 216 MHz
  APB1        : /4  → 54 MHz
  APB2        : /2  → 108 MHz
```

### Step 2 — I2C1 (IMU)
```
Connectivity → I2C1:
  Mode           : I2C
  Speed Mode     : Fast Mode
  Clock Speed    : 400000 (400 kHz)
Pins auto-assign: PB8 (SCL), PB9 (SDA) ✅
```

### Step 3 — USART6 (Raspberry Pi)
```
Connectivity → USART6:
  Mode           : Asynchronous
  Baud Rate      : 2000000
  Word Length    : 8 Bits
  Stop Bits      : 1
  Parity         : None
  Hardware Flow  : Disable
Pins: PC6 (TX = D1), PC7 (RX = D0) ✅
```

### Step 4 — TIM11 (LEFT RPWM → A4/PF7)
```
Timers → TIM11:
  Channel 1      : PWM Generation CH1
  PSC            : 0
  ARR            : 10799   (216MHz/10800 = 20kHz)
  Auto-reload    : Enable
  Mode           : PWM mode 1
  Polarity       : High
Pin: PF7 (A4) ✅
```

### Step 5 — TIM10 (LEFT LPWM → A5/PF6)
```
Timers → TIM10:
  Channel 1      : PWM Generation CH1
  PSC            : 0
  ARR            : 10799
  Auto-reload    : Enable
  Mode           : PWM mode 1
  Polarity       : High
Pin: PF6 (A5) ✅
```

### Step 6 — TIM14 (RIGHT RPWM → A2/PF9)
```
Timers → TIM14:
  Channel 1      : PWM Generation CH1
  PSC            : 0
  ARR            : 10799
  Auto-reload    : Enable
  Mode           : PWM mode 1
  Polarity       : High
Pin: PF9 (A2) ✅
```

### Step 7 — TIM13 (RIGHT LPWM → A3/PF8)
```
Timers → TIM13:
  Channel 1      : PWM Generation CH1
  PSC            : 0
  ARR            : 10799
  Auto-reload    : Enable
  Mode           : PWM mode 1
  Polarity       : High
Pin: PF8 (A3) ✅
```

### Step 8 — Encoder GPIO Interrupts

In Pinout view, click each pin and assign:

```
PA0  (A0)  → GPIO_EXTI0   FL encoder
PB4  (D3)  → GPIO_EXTI4   FR encoder
PA15 (D9)  → GPIO_EXTI15  RL encoder
PG6  (D2)  → GPIO_EXTI6   RR encoder
```

For each pin in GPIO settings:
```
GPIO mode  : External Interrupt - Rising edge trigger
GPIO Pull  : Pull-down
```

In NVIC tab, enable:
```
☑ EXTI line0 interrupt          (PA0  FL)
☑ EXTI line4 interrupt          (PB4  FR)
☑ EXTI lines[9:5] interrupt     (PG6  RR)
☑ EXTI lines[15:10] interrupt   (PA15 RL)
```

### Step 9 — Generate Code
```
Project → Settings:
  Project Name   : terrain_bot
  Toolchain/IDE  : STM32CubeIDE
  
Code Generator:
  ☑ Copy only necessary library files
  ☑ Generate peripheral initialization as .c/.h files

→ GENERATE CODE → Open in STM32CubeIDE
```

---

## 4. CubeIDE Integration

### Copy firmware files
Copy all files from repo `Core/Src/` and `Core/Inc/` into your generated project.

### Add to main.c
```c
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
App_Run();   /* never returns */
/* USER CODE END 2 */
```

### Add EXTI handlers to stm32f7xx_it.c
```c
/* USER CODE BEGIN EXTI0_IRQn 1 */
/* handled by HAL_GPIO_EXTI_Callback in app.c */
/* USER CODE END EXTI0_IRQn 1 */
```

The `HAL_GPIO_EXTI_Callback` is already in `app.c`:
```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_0:   Encoder_PulseISR(ENC_FL); break; // A0 PA0
        case GPIO_PIN_4:   Encoder_PulseISR(ENC_FR); break; // D3 PB4
        case GPIO_PIN_15:  Encoder_PulseISR(ENC_RL); break; // D9 PA15
        case GPIO_PIN_6:   Encoder_PulseISR(ENC_RR); break; // D2 PG6
        default: break;
    }
}
```

### Link micro-ROS library
Follow the micro-ROS STM32 build guide to generate and link `libmicroros.a`.

---

## 5. Raspberry Pi Setup

### Install ROS2 Humble
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install micro-ROS agent
```bash
sudo apt install -y ros-humble-micro-ros-agent
```

### Create udev rule for TTL cable
```bash
# Find cable
ls /dev/ttyUSB*

# Create rule (CP2102 example)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="stm32"' \
  | sudo tee /etc/udev/rules.d/99-stm32.rules
sudo udevadm control --reload-rules
```

### Start micro-ROS agent
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm32 -b 2000000
```

---

## 6. First Boot Sequence

```
1. Connect all hardware
2. Start micro-ROS agent on RPi
3. Power STM32 via USB
4. LCD shows [CALIB] — keep robot STILL for ~2 seconds
5. LCD shows [RUN]   — robot is live!
6. LCD displays live IMU angles + wheel speeds + risk bar
```

### Verify topics
```bash
ros2 topic list
# /terrain_bot/imu/data
# /terrain_bot/wheel_ticks
# /terrain_bot/wheel_velocity

ros2 topic hz /terrain_bot/imu/data       # ~100 Hz
ros2 topic hz /terrain_bot/wheel_velocity # ~50 Hz
```

---

## 7. Motor Testing

```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.030}, angular: {z: 0.0}}" --once

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## 8. LCD Dashboard

The 4.3" built-in display shows live data at 10 Hz:

```
┌──────────────────────────────────────────────────────────────┐
│ STM32 TERRAIN BOT    RISK:[████░░░░]            [ RUN ]      │
├───────────────────────┬──────────────────────────────────────┤
│ IMU ORIENTATION       │ WHEEL SPEEDS (mm/s)                  │
│ Roll : +012.3°        │ FL [████████████░] +32.1             │
│ ░░orange░░░░░░░       │ FR [████████░░░░░] +22.5             │
│ Pitch: -005.7°        │ RL [████████████░] +31.8             │
│ ░░blue░░░░░░░░        │ RR [████████░░░░░] +22.1             │
│ Yaw  : +089.1°        │                                      │
│ ░░purple░░░░░░        │                                      │
├───────────────────────┴──────────────────────────────────────┤
│ PITCH SAFETY  [████░░░░░░░░░░░░░░░░░░░░]  -005.7°           │
└──────────────────────────────────────────────────────────────┘
```

Colour coding:
- 🟢 Green  = safe / forward motion
- 🟡 Amber  = caution (pitch >15° or risk >30%)
- 🔴 Red    = danger (pitch >25° or risk >70%) or reverse
- 🔵 Blue   = pitch bar
- 🟠 Orange = roll bar
- 🟣 Purple = yaw bar

---

## 9. Safety Layer Tuning

All in `Core/Src/app.c`:
```c
#define SAFETY_THETA_SAFE   10.0f  // Below = full speed
#define SAFETY_THETA_CRIT   30.0f  // Above = E-stop
#define SAFETY_ESTOP_PITCH  30.0f  // Hard stop pitch
#define SAFETY_ESTOP_ROLL   40.0f  // Hard stop roll
#define SAFETY_LPF_BETA     0.90f  // Smoothing factor
```

---

## 10. Phase 2 — When MCP23017 Arrives

Connect MCP23017 to CN2:
```
MCP23017    CN2 Pin
VCC      →  Pin 5 (+3V3)
GND      →  Pin 7 (GND)
SDA      →  Pin 1 (PB9)
SCL      →  Pin 3 (PB8)
A0,A1,A2 →  GND (address 0x20)
RESET    →  Pin 5 (+3V3)
INTA     →  D5 (PI0) STM32 interrupt
INTB     →  D7 (PI3) STM32 interrupt

GPA0 → FL encoder CH_B
GPA1 → FR encoder CH_B
GPA2 → RL encoder CH_B
GPA3 → RR encoder CH_B
```

This adds full quadrature counting for accurate odometry and SLAM.

---

## 11. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| LCD stuck on CALIB | micro-ROS agent not running | Start agent on RPi first |
| No /topics on RPi | UART wiring wrong | Swap D0/D1 wires |
| Motors don't move | BTS7960 EN pins | Tie R_EN + L_EN to 3.3V |
| Motor runs one direction only | RPWM/LPWM swapped | Swap A4↔A5 or A2↔A3 |
| Encoder not counting | Pull-down missing | Set GPIO Pull-Down in CubeMX |
| IMU init fails | I2C wiring | Check CN2 pin 1=SDA, pin 3=SCL |
| Speed always 0 | EXTI not enabled | Enable NVIC in CubeMX |
| Robot tips and stops | Safety E-stop | Place flat, recalibrate |
