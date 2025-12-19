---
title: "Physics Simulation with Gazebo"
sidebar_label: "Chapter 1: Physics Simulation"
description: "Learn physics-based simulation and digital twins for humanoid robots using Gazebo"
keywords: [digital-twin, gazebo, physics, simulation, robotics]
---

# Physics Simulation with Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the role of digital twins in Physical AI
- Configure physics engines, gravity, and collisions in Gazebo
- Create and use world files for simulation environments
- Spawn and simulate humanoid robots in Gazebo
- Troubleshoot common physics simulation issues

## Prerequisites

Before starting this chapter, you should:
- Have completed Module 1 (ROS 2 Fundamentals)
- Have Gazebo Classic installed with ROS 2 Humble integration
- Understand basic ROS 2 concepts (nodes, topics, parameters)
- Have basic knowledge of robot kinematics

## What are Digital Twins in Physical AI?

A digital twin is a virtual representation of a physical system that mirrors its real-world behavior in simulation. In the context of Physical AI and robotics, digital twins serve as:

1. **Development Environments**: Test algorithms without risking physical hardware
2. **Training Platforms**: Train AI models with unlimited data
3. **Validation Systems**: Verify robot behaviors before deployment
4. **Debugging Tools**: Isolate and fix issues in a controlled environment

Digital twins are particularly valuable in robotics because they allow for:
- Rapid iteration without hardware constraints
- Safe testing of control algorithms
- Generation of large datasets for machine learning
- Reproducible experiments

## Understanding Gazebo Physics Simulation

Gazebo is a 3D simulation environment that provides accurate physics simulation, high-quality graphics, and convenient programmatic interfaces. At its core, Gazebo uses physics engines to simulate real-world physics including:

- **Gravity**: Constant downward acceleration (typically 9.81 m/s²)
- **Collisions**: Detection and response when objects make contact
- **Friction**: Resistance to motion between surfaces
- **Inertia**: Resistance to changes in motion based on mass distribution

### Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different characteristics:

- **ODE (Open Dynamics Engine)**: Default and most stable engine, good for most robotics applications
- **Bullet**: Faster but less stable for complex scenarios
- **DART**: More advanced but not available in Gazebo Classic

For humanoid robot simulation, ODE provides the best balance of stability and accuracy.

### Configuring Physics Parameters

Physics parameters in Gazebo are defined in world files using the `<physics>` tag:

```xml
<physics name="1ms" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

Key parameters include:
- `max_step_size`: Time step for physics integration (smaller = more accurate but slower)
- `real_time_factor`: Target simulation speed relative to real time
- `real_time_update_rate`: Updates per second (1/dt where dt is step size)
- `gravity`: Gravitational acceleration vector

## Creating World Files

World files define the simulation environment including physics parameters, lighting, and static objects. They use SDF (Simulation Description Format) which is based on XML.

### Basic World Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Physics configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Adding Objects to Your World

You can add various objects to your simulation world:

```xml
<!-- Adding a simple box -->
<model name="simple_box">
  <pose>2 2 0.5 0 0 0</pose>
  <static>false</static>
  <link name="link">
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.083</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.083</iyy>
        <iyz>0</iyz>
        <izz>0.083</izz>
      </inertia>
    </inertial>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

## Spawning Robots in Gazebo

Robots are typically defined using URDF (Unified Robot Description Format) and then converted to SDF for Gazebo. You can spawn robots in several ways:

### Using ROS 2 Launch Files

Create a launch file to spawn your robot:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with a world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('my_robot_gazebo'),
                    'worlds',
                    'my_world.world'
                ])
            }.items()
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        )
    ])
```

### Using Command Line

You can also spawn robots directly from the command line:

```bash
# Launch Gazebo with a world file
ros2 launch gazebo_ros empty_world.launch.py world_name:=path/to/my_world.world

# In another terminal, spawn a robot
ros2 run gazebo_ros spawn_entity.py -file path/to/robot.urdf -entity my_robot -x 0 -y 0 -z 0.5
```

## Example: Simulating a Humanoid in Gazebo

Let's create a complete example of simulating a humanoid robot:

### 1. Create a Basic World File

Create `basic_humanoid_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_world">
    <physics name="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add some obstacles for the humanoid to navigate around -->
    <model name="obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 2. Launch the Simulation

```bash
# Launch Gazebo with your world
ros2 launch gazebo_ros empty_world.launch.py world_name:=basic_humanoid_world.world

# In another terminal, spawn a humanoid robot (assuming you have one available)
# For example, if you have a simplified NAO-like robot from Module 1:
ros2 run gazebo_ros spawn_entity.py -file path/to/humanoid.urdf -entity humanoid_robot -x 0 -y 0 -z 0.5
```

## Troubleshooting Physics Simulation

Common issues and solutions:

### 1. Robot Falls Through Ground
- Check that collision geometries are properly defined
- Ensure the robot is positioned above the ground (z > 0)
- Verify static flag is set for ground plane

### 2. Unstable Physics (Jittering, Exploding)
- Reduce `max_step_size` in physics configuration
- Check mass and inertia values in URDF/SDF
- Ensure proper joint limits and dynamics

### 3. Slow Simulation
- Increase `max_step_size` (trade accuracy for speed)
- Reduce complexity of collision meshes
- Lower `real_time_update_rate`

## Summary

In this chapter, you've learned:
- The fundamental concepts of digital twins in Physical AI
- How Gazebo provides physics simulation for robotics
- How to configure physics parameters and create world files
- How to spawn and simulate humanoid robots in Gazebo
- Common troubleshooting techniques for physics simulation

This foundation prepares you for the next chapter where you'll learn about sensor simulation in Gazebo.