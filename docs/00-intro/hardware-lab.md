---
sidebar_position: 2
title: The Hardware Lab
---

# Your Physical AI Lab Setup

To fully engage with this curriculum, you will need access to specific hardware. This guide details the recommended setup, from the powerful workstation that will run your simulations to the physical robot itself.

## 1. The Digital Twin Rig (Workstation)

Your primary workstation is where you will build, train, and simulate your AI agents. This is the "brain" of your digital twin environment.

-   **Operating System:** Ubuntu 22.04 LTS (Recommended for ROS 2 and NVIDIA Isaac Sim compatibility).
-   **GPU:** NVIDIA GeForce **RTX 4070 Ti** or higher. A powerful, modern GPU is non-negotiable for photorealistic simulation and training AI models.
-   **CPU:** Modern multi-core processor (e.g., Intel Core i7/i9 or AMD Ryzen 7/9).
-   **RAM:** 32GB DDR4 or higher.

## 2. The Nervous System (Edge Computer)

Once your AI is trained in simulation, you'll deploy it to an edge device that can be mounted on a physical robot.

-   **Recommended Device:** **NVIDIA Jetson Orin Nano Developer Kit**.
-   **Why?** The Jetson platform provides powerful GPU acceleration in a small, low-power form factor, making it ideal for running real-time inference and control loops directly on the robot.

## 3. The Senses (Sensors)

Your robot needs to perceive the world. While simulations are a starting point, working with real sensor data is crucial.

-   **Depth & Vision:** **Intel RealSense Depth Camera** (e.g., D435i or D455). This provides both a standard RGB video stream and a depth map, allowing your robot to perceive distances and navigate obstacles.
-   **Spatial Awareness:** **LIDAR (Light Detection and Ranging)**. A 2D or 3D LIDAR sensor is essential for accurate mapping (SLAM) and localization.

## 4. The Body (Robots)

This curriculum is designed around two primary robot platforms from Unitree Robotics.

### Unitree Go2 (Quadruped)

The Go2 is a highly agile, dog-like robot. It's an excellent platform for learning about:
-   Dynamic locomotion and stability (walking, running, jumping).
-   Navigation and mapping in complex environments.
-   Basic sensor integration.

### Unitree G1 (Humanoid)

The G1 is a human-like robot and represents the capstone platform for this course. With the G1, you will explore:
-   Bipedal locomotion (walking on two legs).
-   Complex manipulation tasks with arms and hands.
-   Human-Robot Interaction (HRI).

While physical access to these robots is ideal, a majority of this course can be completed using their simulated "digital twin" counterparts in Isaac Sim.
