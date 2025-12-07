---
sidebar_position: 1
title: Introduction to Physical AI
---

# Introduction to Physical AI

Welcome to the frontier of artificial intelligence. This textbook is your guide to **Embodied Intelligence**, the paradigm shift that moves AI from purely digital realms into the physical world.

## Embodied Intelligence vs. Digital AI

**Digital AI**, the kind you might be familiar with (like ChatGPT, Midjourney, or a stock-trading algorithm), exists purely as software. It processes data—text, images, numbers—and produces data as output. While incredibly powerful, it lacks a body to interact with the physical world. It cannot pick up a cup, open a door, or navigate a crowded room.

**Embodied Intelligence**, or **Physical AI**, gives AI a body. It connects algorithms to sensors and actuators, allowing the AI to perceive, reason about, and act within a physical environment. This is the core of robotics.

Key differences:

| Aspect | Digital AI | Embodied (Physical) AI |
| :--- | :--- | :--- |
| **Environment** | Virtual, data-based | Real world, physics-based |
| **Input** | Text, images, structured data | Sensor data (LIDAR, cameras, IMU) |
| **Output**| Text, images, predictions | Physical actions (motor torque, movement) |
| **Challenge** | Logic, knowledge, patterns | Uncertainty, latency, safety, physics |

## The Humanoid Landscape

The dream of autonomous humanoid robots is rapidly becoming a reality. Several key players are pushing the boundaries of what's possible:

*   **Boston Dynamics:** Famous for `Atlas`, a robot that can run, jump, and perform complex gymnastic routines. Their work demonstrates incredible feats of dynamic stability and control.
*   **Tesla:** Developing `Optimus`, a general-purpose humanoid robot designed for manufacturing and eventually, everyday tasks. Their focus is on scalable production and real-world AI.
*   **Unitree Robotics:** Known for their agile quadruped (dog-like) robots like the `Go2` and their emerging humanoid, the `G1`. They provide a powerful and relatively accessible platform for researchers and developers.

## Understanding Physics in AI

When an AI has a body, it is bound by the laws of physics. This is a fundamental constraint that digital AI never has to consider. Key concepts we will explore include:

-   **Dynamics:** The study of forces and their effect on motion. How much torque is needed to lift an arm?
-   **Kinematics:** The study of motion without considering the forces that cause it. What sequence of joint angles will place the robot's hand at a specific coordinate?
-   **Gravity & Friction:** Constant forces that the robot's control systems must continuously fight or leverage.
-   **Collision:** Detecting and reacting to contact with the world, which is critical for safety and manipulation.

Throughout this course, you will learn to model and simulate these physical principles, and then deploy your AI agents to control robots in the real world.
