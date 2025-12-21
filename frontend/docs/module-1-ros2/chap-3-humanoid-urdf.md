---
title: Humanoid Robot Modeling with URDF
sidebar_label: Chapter 3 - Humanoid Robot Modeling with URDF
---

# Chapter 3: Humanoid Robot Modeling with URDF

## Introduction

This chapter covers the Unified Robot Description Format (URDF), which is used to describe robot models in ROS. URDF is an XML-based format that describes robot properties including kinematic and dynamic properties, visual representation, and collision model. Understanding URDF is essential for working with humanoid robots, as it provides a standardized way to represent their complex structure and kinematic properties.

## Concepts

### URDF Overview

URDF (Unified Robot Description Format) is an XML format that describes robot models. It includes:
- Kinematic structure (links and joints)
- Visual and collision properties
- Inertial properties
- Material properties

URDF allows ROS tools to understand the physical structure of a robot, enabling simulation, visualization, motion planning, and other robotics applications.

### Links and Joints

In URDF, a robot is described as a collection of rigid bodies (links) connected by joints:

#### Links
Links represent rigid bodies with the following properties:
- **Visual**: How the link appears in visualization tools
- **Collision**: How the link interacts in collision detection
- **Inertial**: Physical properties like mass and moment of inertia

#### Joints
Joints connect links and define their kinematic relationship:
- **Joint Types**: Fixed, continuous, revolute, prismatic, floating, planar
- **Joint Limits**: Range of motion constraints
- **Joint Origins**: Position and orientation relative to parent link

### Humanoid Robot Modeling

Humanoid robots have a specific structure that typically includes:
- Head, torso, and base
- Two arms with shoulders, elbows, and wrists
- Two legs with hips, knees, and ankles
- Proper joint types and limits to enable human-like movement

The kinematic structure of a humanoid robot is usually represented as a tree with the base/torso as the root, with limbs branching out.

## Examples

### Basic URDF Structure

Here's a minimal URDF example showing the basic structure:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="fixed_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
  </joint>

  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

### Simple Humanoid Robot URDF

Here's a more complex example of a simplified humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Arm (similar to left) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Right Leg (similar to left) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>
</robot>
```

### Reading and Interpreting URDF Files

When reading a URDF file, focus on:

1. **Link Structure**: Identify the main body parts and their connections
2. **Joint Types**: Understand how parts can move relative to each other
3. **Joint Limits**: Know the range of motion for each joint
4. **Physical Properties**: Mass, dimensions, and inertial properties
5. **Kinematic Chain**: Trace the connections from base to end effectors

## Summary

This chapter covered the fundamentals of URDF for describing robot models, with a focus on humanoid robot modeling. You now understand:
- The structure of URDF files with links and joints
- How to represent the kinematic structure of humanoid robots
- The properties that define each link (visual, collision, inertial)
- How joints connect links and define their movement constraints
- How to read and interpret URDF files for humanoid robots

URDF is essential for robot simulation, visualization, and motion planning. The examples provided demonstrate how to create and understand URDF models for humanoid robots, which form the basis for more complex robotic applications.

## Practical Exercises

1. Create a simple URDF file for a 2-link robot arm with one revolute joint.
2. Modify the humanoid URDF example to add feet to the lower leg links.
3. Use a URDF visualization tool to view your robot models and verify their structure.