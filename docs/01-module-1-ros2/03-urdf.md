---
sidebar_position: 3
---

# Defining the Body: Introduction to URDF

## The Robot's Blueprint

URDF stands for **Unified Robot Description Format**, and it's essentially the **blueprint of your robot**. Just as architects use blueprints to define the structure of buildings, roboticists use URDF to define the physical structure of robots. Think of URDF as the skeleton that gives your robot its shape and mechanical properties.

## Links vs Joints: The Building Blocks

In URDF, robots are constructed using two fundamental elements:

### Links: The Bones
- **Links** are the rigid bodies of your robot, like bones in a skeleton
- Each link has physical properties like mass, inertia, and visual/collision geometry
- Links are the **static components** of your robot (e.g., chassis, arm segments, wheels)

### Joints: The Muscles and Connectors
- **Joints** connect links together, like joints in your body
- Joints define how links can move relative to each other
- Joints are the **dynamic components** that enable movement (e.g., hinges, sliders, ball joints)

Here's a simple conceptual model:

```
[LINK: Robot Base] ← [JOINT: Revolute Joint] → [LINK: Rotating Arm] ← [JOINT: Hinge] → [LINK: Gripper]
```

## Anatomy of a URDF File

A typical URDF file has this structure:

```xml
<?xml version="1.0"?>                    <!-- XML declaration -->
<robot name="my_robot">                   <!-- Robot definition -->
  <!-- Links: The rigid bodies -->
  <link name="base_link">                 <!-- Base link definition -->
    <visual>                              <!-- Visual properties -->
      <geometry>                          <!-- Shape definition -->
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>         <!-- Blue color -->
      </material>
    </visual>
    <collision>                           <!-- Collision properties -->
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>                            <!-- Inertial properties -->
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints: The connections -->
  <joint name="base_to_wheel" type="continuous">  <!-- Continuous joint (can rotate freely) -->
    <parent link="base_link"/>            <!-- Parent link -->
    <child link="wheel_link"/>            <!-- Child link -->
    <origin xyz="0 0.25 -0.05" rpy="0 0 0"/>  <!-- Position and orientation -->
    <axis xyz="0 1 0"/>                   <!-- Rotation axis -->
  </joint>

  <link name="wheel_link">                <!-- Wheel link definition -->
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Sample URDF: Simple Robot Arm

Here's a complete URDF example for a simple 2-DOF (Degree of Freedom) robot arm:

```xml linenums="1"
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base of the robot (immovable) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- First arm segment -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Second arm segment -->
  <link name="lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- Joint connecting base to upper arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>                   <!-- Rotate around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Joint connecting upper arm to lower arm -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>   <!-- Positioned at end of upper arm -->
    <axis xyz="0 1 0"/>                   <!-- Rotate around Y-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

</robot>
```

### Code Explanation Line by Line:

- **Line 1**: XML declaration specifying version
- **Line 2**: Robot element with name "simple_arm"
- **Lines 4-18**: Base link definition with visual, collision, and inertial properties
- **Line 6**: Visual element defining how the link appears
- **Line 7**: Geometry definition (cylinder shape)
- **Lines 9-11**: Material definition with gray color
- **Lines 12-14**: Collision properties for physics simulation
- **Lines 15-17**: Inertial properties for dynamics
- **Lines 20-34**: Upper arm link definition
- **Lines 36-50**: Lower arm link definition
- **Lines 52-60**: Shoulder joint connecting base to upper arm
- **Line 54**: Parent link (which link comes before in the chain)
- **Line 55**: Child link (which link comes after in the chain)
- **Line 56**: Origin specifying position and orientation of the joint
- **Line 57**: Axis around which the joint rotates
- **Line 58**: Joint limits for range of motion
- **Lines 62-70**: Elbow joint connecting upper arm to lower arm

## The Tree Structure of Robots

Robots in URDF follow a **tree structure** (no loops), where each link (except the root) has exactly one parent. Here's how the simple arm structure looks:

```
base_link (root)
├── shoulder_joint (revolute)
    └── upper_arm
        └── elbow_joint (revolute)
            └── lower_arm
```

This tree structure ensures:
- **Unique path** from any link to the root
- **No kinematic loops** (which would complicate inverse kinematics)
- **Clear parent-child relationships** for transformations

## Key URDF Elements

### Visual Properties
Defines how the robot looks in simulation and visualization tools:
- **Geometry**: Shape (box, cylinder, sphere, mesh)
- **Material**: Color and texture
- **Origin**: Position and orientation offset

### Collision Properties
Defines how the robot interacts with other objects:
- **Geometry**: Shape used for collision detection
- Often similar to visual geometry but can be simplified for performance

### Inertial Properties
Defines physical properties for dynamics simulation:
- **Mass**: How heavy the link is
- **Inertia**: How mass is distributed (resistance to rotation)

### Joint Properties
Defines how links can move relative to each other:
- **Type**: revolute, continuous, prismatic, fixed, etc.
- **Limits**: Range of motion, max effort, max velocity
- **Axis**: Direction of motion

## Joint Types

1. **Fixed**: No movement (used for permanently attached parts)
2. **Revolute**: Rotational movement with limits (like an elbow)
3. **Continuous**: Rotational movement without limits (like a wheel)
4. **Prismatic**: Linear sliding movement (like a piston)
5. **Floating**: 6 DOF movement (rarely used)

## URDF Best Practices

:::tip
**Best Practices for URDF Development:**
- Start with a simple model and gradually add complexity
- Use consistent naming conventions (e.g., `link_name`, `joint_name`)
- Validate your URDF with tools like `check_urdf` or `xacro`
- Separate complex URDFs into multiple files using `<xacro>` macros
- Include realistic inertial properties for accurate simulation
:::

:::danger
**Common URDF Mistakes:**
- Creating kinematic loops (multiple paths from child to parent)
- Missing parent-child relationships
- Incorrect units (always use meters, kilograms, seconds)
- Unrealistic inertial properties (mass of 0 or negative values)
- Inconsistent coordinate frames between links
:::

## Visualization Tools

Once you've created your URDF, you can visualize it using:
- **RViz**: ROS visualization tool
- **Gazebo**: Physics simulation environment
- **URDF viewer**: Standalone tools for quick visualization
- **Blender**: With appropriate plugins

## Summary

URDF is the foundational language for describing robot geometry and kinematics in ROS 2. By understanding links (bones) and joints (connections), you can create accurate representations of your physical robots. The tree structure ensures mathematical simplicity while allowing for complex robot designs. With proper URDF files, you can simulate, visualize, and control your robots effectively in the ROS 2 ecosystem.