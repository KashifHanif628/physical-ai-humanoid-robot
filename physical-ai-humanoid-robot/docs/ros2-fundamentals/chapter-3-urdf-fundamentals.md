---
title: "Humanoid URDF Fundamentals"
sidebar_label: "Chapter 3: URDF Fundamentals"
description: "Learn about Unified Robot Description Format (URDF) and humanoid robot models"
keywords: [ros2, urdf, robot, model, humanoid, description, format]
---

# Humanoid URDF Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF files
- Create and modify simple humanoid robot models
- Visualize URDF models in ROS 2
- Connect URDF models with ROS 2 control systems

## Prerequisites

Before starting this chapter, you should:
- Have completed Chapters 1 and 2 (ROS 2 basics and Python control)
- Understand basic ROS 2 concepts (nodes, topics, parameters)
- Have a working ROS 2 environment

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robots. It defines the physical and visual properties of a robot, including:

- Links: Rigid parts of the robot
- Joints: Connections between links
- Materials: Visual appearance properties
- Transmissions: How joints are controlled
- Gazebo elements: Simulation-specific properties

### Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Define joints -->
  <joint name="base_to_top" type="fixed">
    <parent link="base_link"/>
    <child link="top_link"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="top_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF Elements Explained

### Links

Links represent rigid parts of the robot. Each link has:
- A name (unique identifier)
- Visual properties (shape, color, material)
- Collision properties (for physics simulation)
- Inertial properties (mass, center of mass, inertia tensor)

```xml
<link name="link_name">
  <visual>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1"/>
    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
  </inertial>
</link>
```

### Joints

Joints connect links and define how they can move relative to each other. Joint types include:
- `revolute`: Rotational joint with limits
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement between links
- `floating`: 6-DOF joint
- `planar`: Movement in a plane

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Simplified NAO-like Humanoid Robot Model

Let's create a simplified version of a NAO-like humanoid robot that focuses on key humanoid features:

```xml
<?xml version="1.0"?>
<robot name="simplified_nao">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.25"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.175"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_to_right_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.1 0 0.1" rpy="0 0 -1.57"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="right_upper_arm_to_lower_arm" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="torso_to_left_upper_arm" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 1.57"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="left_upper_arm_to_lower_arm" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="torso_to_right_upper_leg" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.05 0 -0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="right_upper_leg_to_lower_leg" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="torso_to_left_upper_leg" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.05 0 -0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.15" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="left_upper_leg_to_lower_leg" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Visualizing URDF Models

To visualize your URDF model in ROS 2, you can create a simple Python script that publishes the robot description to the `/robot_description` topic and uses RViz for visualization.

## Connecting URDF with ROS 2 Control

URDF models can be connected to ROS 2 control systems using the ROS 2 Control framework. This allows you to command joint positions, velocities, or efforts.

## Modifying URDF Properties

URDF files can be modified to change the robot's appearance and physical properties:

- Change link dimensions in the `<geometry>` tags
- Adjust joint limits in the `<limit>` tags
- Modify colors in the `<material>` tags
- Update inertial properties for physics simulation

## Summary

In this chapter, you've learned:
- The structure and components of URDF files
- How to create a simplified humanoid robot model
- The different types of joints and their properties
- How URDF connects with ROS 2 control systems

This completes our exploration of ROS 2 fundamentals, from basic concepts to advanced topics like URDF modeling.