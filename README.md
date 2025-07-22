# 🚗 Autonomous Mobile Robot (AMR) Test Platform

This repository contains the code and documentation for a modular AMR test platform built to validate real-world navigation, localization, and safety features. The system integrates multiple sensors and compute elements including the **Jetson Nano**, **mmWave radar**, **IMU + GPS**, and **ESP32** microcontrollers to support advanced robotics development using **ROS 2** and **Micro-ROS**.

---
## 🤖 AMR Test Plaform
<p align="center">
  <img src="Images/AMR_Test_Platform.jpg" alt="AMR TEST PLATFORM" width="600"/>
</p>

## 📌 Features

- ROS 2-based architecture (tested with Jazzy and Humble)
- SLAM and autonomous navigation (Nav2 stack)
- Sensor fusion with mmWave radar, GPS, IMU, and Cameras.
- BLDS Motor control using ESC via ESP32
- Real-time battery monitoring and system health feedback
- Modular design for rapid testing and iteration
- Edge compute with Jetson Nano for onboard AI & vision tasks

---

## 🛠️ Hardware Overview

| Component            | Description                                         |
|---------------------|-----------------------------------------------------|
| Jetson Nano         | Main compute unit running ROS 2 and AI processing   |
| AWR1843 mmWave Radar| Obstacle detection and mapping                      |
| Camera System  | Used for collecting test footage and also object detection                      |
| ESP32 x2            | Micro-ROS nodes for motor/LED control and sensors   |
| BLDC Motor Driver     | TRAXXAS Truck BLDC motor and ESC           |
| IMU + GPS Module    | Used for localization and dead-reckoning fusion     |
| Battery Monitor     | Custom circuit for voltage and current feedback     |
| Chassis             | TRAXXAS MAXX 4S 4WD Chassis      |

---

## 🧠 Circuit Design
<p align="center">
  <img src="Images/AMR_Circuit_Diagram.jpg" alt="Circuit Diagram" width="600"/>
</p>

## 📐 CAD Design
<p align="center">
  <img src="Images/AMR_CAD.jpg" alt="3D CAD" width="600"/>
</p>

## 🧰 Tests
<p align="center">
  <img src="Images/AMR_Test.jpg" alt="Testing Results" width="800"/>
</p>
