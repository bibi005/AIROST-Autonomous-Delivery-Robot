
# Advanced Autonomous Delivery Robot

### AIROST 2026/2027 – IoT/ROS Challenge

**Team Name:** Stardust Crusaders

**Team Members:**
- Shima Jamal Idris
- Onab Siddig

---

## 1. Project Overview

The Advanced Autonomous Delivery Robot is designed to autonomously navigate a predefined track, identify a designated ping-pong ball using colour-coded instructions, pick it up, deliver it to the correct station, and return to the designated base.

The project aims to develop a fully autonomous robotic system using embedded control, sensor-based navigation, colour recognition, and servo-controlled object handling.

## 2. Objectives

- Develop an autonomous line-following robot.
- Detect and count track junctions for navigation.
- Identify colour-coded instructions.
- Navigate to and pick up the designated ping-pong ball.
- Deliver the ball to the correct station.
- Return autonomously to the assigned base.

## 3. Hardware Components

| Component | Purpose | Status |
|---|---|---|
| STM32F103C8T6 Blue Pill | Main microcontroller | Selected |
| 2 × N20 geared DC motors | Differential-drive movement | Selected |
| TB6612FNG | Dual motor driver | Selected |
| 5-channel IR sensor array | Line following and junction detection | Selected |
| TCS34725 colour sensor | Reads colour-coded instructions | Selected |
| MPU6050 | Gyroscope-assisted turning | Tested |
| Servo motor and front claw | Ball pickup and delivery | SG90 Servo |
| RGB LED module | Battery-level indication | Planned; requires battery-voltage monitoring |
| Passive buzzer | Audio indication | Marks the start and end of the challenge and the completions of tasks |
| Battery and regulator | Motor and logic power | Selected |

## 4. Software and Development Tools

- STM32CubeIDE
- STM32CubeProgrammer
- ST-Link
- Embedded C
- GitHub

## 5. System Architecture

The STM32 microcontroller acts as the central controller. It processes IR sensor readings for line following and junction detection, reads colour instructions using the colour sensor, and controls the motors and servo claw.

Battery voltage is intended to be measured through a suitable voltage-divider circuit connected to an STM32 ADC input. The RGB LED module will indicate battery level based on the measured voltage. The voltage-divider values, ADC scaling and battery warning thresholds must be calculated for the final battery configuration and tested before this feature is considered implemented.

## 6. Battery-Level Indication

The RGB LED is dedicated to battery-level indication, not general robot status.

Proposed indication:

- **Green:** Battery level is in the normal operating range.
- **Yellow:** Battery level is getting low.
- **Red:** Battery voltage is critically low.

The exact voltage thresholds depend on the final battery chemistry, cell count and manufacturer limits. They must be calculated and verified before use.

## 7. Repository Structure

```text
AIROST-Autonomous-Delivery-Robot/
│
├── README.md
├── Documentation/
├── Firmware/
├── Hardware/
├── Mechanical/
├── Testing/
└── Media/
```

## 8. Development Status

**Current Stage:** Software development

Firmware implementation, hardware integration, sensor calibration, battery-voltage monitoring and autonomous mission testing are in progress or pending.

## 9. Expected Outcome

A fully autonomous delivery robot capable of completing the advanced AIROST challenge, including colour-instruction recognition, ball pickup, delivery and return to the correct base.

---

**Team Stardust Crusaders**  
AIROST 2026/2027 – IoT/ROS
