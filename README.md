# Gesture-Controlled-Robotic-Arm

This system enables **real-time control of a robotic arm** using hand gestures detected via **MediaPipe**. The gestures are processed in Python and translated into control commands, which are transmitted over UART to an **STM32 microcontroller**.

The STM32 is programmed using **CMSIS** and **HAL drivers**, with hardware timers configured in **PWM mode** to control servo motors. Each timer channel generates pulse widths (typically 0.5–2.5 ms within a 20 ms period) corresponding to specific joint angles.

The microcontroller continuously reads UART data, decodes the incoming gesture commands, and updates PWM outputs accordingly. All these tasks are controlled using **FreeRTOS** to ensure deterministic execution.

This project demonstrates the integration of **vision-based HMI** with embedded motor control using **timer-driven PWM** and **efficient serial communication**.

---

## Features

- Detects multiple hand gestures:
  - Fist
  - Open Hand
  - Thumbs Up
  - Peace
  - Point
  - Rock
  - OK
- Sends a **gesture code** to the STM32 for mapping or control.
- Real-time display with gesture annotations.
- Full integration with STM32 PWM control using FreeRTOS.

---
## Requirements

- Python 3.8+
- OpenCV
- MediaPipe
- NumPy
- PySerial
- Webcam
- STM32 board with UART/Serial interface

Install Python dependencies using:

```bash
pip install opencv-python mediapipe numpy pyserial

## Demo

![Hand Gesture Demo](hmi_robot.gif)  


---


