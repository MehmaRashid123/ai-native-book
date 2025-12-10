---
sidebar_position: 4
---

# Navigation 2 (Nav2)

Navigation 2 (Nav2) is the navigation stack for ROS 2, providing path planning, obstacle avoidance, and localization capabilities for mobile robots.

## What You'll Learn

- Nav2 architecture and components
- Path planning algorithms
- Localization in simulated environments
- Integration with Isaac Sim

## Prerequisites

- Understanding of ROS 2 concepts
- Knowledge of SLAM from previous section

## Nav2 Components

Nav2 consists of several key components that work together to enable robot navigation:

### Global Planner
The global planner computes a path from the robot's current location to the goal.

### Local Planner
The local planner executes the global plan while avoiding obstacles in real-time.

### Costmap
The costmap represents obstacles and free space in the robot's environment.