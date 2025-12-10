---
sidebar_position: 3
---

# URDF: Robot Description Format

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot.

## What You'll Learn

- Structure of URDF files
- Defining robot links and joints
- Visual and collision properties
- Creating complex robot models

## Prerequisites

- Understanding of ROS 2 nodes and topics
- Basic knowledge of 3D geometry

## Basic URDF Structure

A simple URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
  </link>
</robot>
```

This defines a simple box-shaped robot with one link.