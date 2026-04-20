# STM32F746G-DISCO Terrain Rover Firmware

This firmware runs on the STM32F746G-DISCO board and acts as the real-time motion controller for a four-wheel skid-steer rover.

The system uses an ESP32 as an encoder co-processor and an ICM-20948 IMU for orientation sensing. The STM32 performs all safety logic, motion control, and ROS communication.

---

## System Overview

The system is split into two embedded processors:

STM32F746G-DISCO:
- Motor PWM generation (4 motors)
- IMU acquisition (ICM-20948)
- Safety monitoring (tilt and vibration)
- Velocity control (per motor)
- ROS communication via micro-ROS
- I2C master reading encoder data from ESP32

ESP32 Encoder Co-Processor:
- Reads all four wheel encoders
- Uses hardware pulse counting
- Calculates velocity
- Responds to STM32 via I2C

---

## Hardware Architecture

### I2C Bus (Shared)

STM32 I2C1 pins:
- PB8 (D15) → SCL
- PB9 (D14) → SDA

Connected devices:
- ICM-20948 IMU (0x68)
- ESP32 encoder module (0x30)

Both devices share the same two-wire bus.

---

## Motor Control

Four DC motors are controlled using BTS7960 drivers.

PWM Outputs:
- PF7 (A4) → TIM11_CH1
- PF6 (A5) → TIM10_CH1
- PF9 (A2) → TIM14_CH1
- PF8 (A3) → TIM13_CH1

PWM frequency: 20 kHz

---

## Encoder Interface (ESP32)

STM32 reads encoder data every 20 ms.

Data received from ESP32 (32 bytes):
- FL ticks
- FR ticks
- RL ticks
- RR ticks
- FL velocity (mm/s)
- FR velocity (mm/s)
- RL velocity (mm/s)
- RR velocity (mm/s)

STM32 also sends motor direction information back to ESP32.

---

## IMU System

Sensor: ICM-20948

Connected via I2C1.

Measured signals:
- Acceleration
- Angular velocity
- Magnetic field

Orientation is calculated using the Madgwick filter.

---

## Communication to ROS

Interface: USART6

Pins:
- PC6 → TX
- PC7 → RX

Baud rate: 2000000

Transport type:
- micro-ROS UART transport

This connects to a Raspberry Pi running the micro-ROS agent.

---

## Main Control Loop Tasks

The firmware runs multiple periodic tasks:

Every 10 ms:
- Read IMU
- Update orientation
- Apply safety logic
- Update motor control

Every 20 ms:
- Read encoder data from ESP32

Every 100 ms:
- Update display

---

## Safety Behaviour

The system continuously monitors robot tilt and vibration.

If excessive tilt is detected:
- Motor speed is reduced
- Motors are stopped if unsafe motion is detected

This logic runs entirely on the STM32 to guarantee fast response.

---

## Build Notes

Required CubeMX configuration:

I2C1:
- PB8 → SCL
- PB9 → SDA
- Speed: 400 kHz

USART6:
- PC6 → TX
- PC7 → RX
- Baud: 2000000

Timers:
- TIM10, TIM11, TIM13, TIM14 in PWM mode
- PWM frequency: 20 kHz

---

## Repository Structure

Core/
- app.c : main application logic
- motor.c : PWM motor control
- encoder.c : ESP32 I2C encoder interface
- icm20948.c : IMU driver
- madgwick.c : orientation filter
- microros_transport.c : UART interface
- lcd_display.c : display control

---

## Design Goal

This firmware is intended for terrain-capable mobile robots where reliability and predictable control behaviour are critical.

The architecture separates sensing, actuation, and communication to ensure stable real-time performance.