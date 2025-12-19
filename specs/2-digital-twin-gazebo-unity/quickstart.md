# Quickstart Guide: Digital Twin Module (Gazebo & Unity)

**Feature**: 2-digital-twin-gazebo-unity
**Version**: 1.0
**Created**: 2025-12-18

## Overview

This quickstart guide provides a complete path for learners to get started with the Digital Twin module. By following this guide, you'll have working Gazebo and Unity simulation environments and run your first examples.

## Prerequisites

Before starting, ensure you have:
- A Linux system (Ubuntu 22.04 recommended) or a virtual machine
- At least 8GB of RAM (16GB recommended for Unity)
- At least 20GB of free disk space
- Internet connection for package downloads
- Basic command line knowledge
- Basic Python knowledge (Python 3.8+)
- Completed Module 1 (ROS 2 Fundamentals)

## Step 1: Install Gazebo Classic

### 1.1 Setup sources for ROS 2 Humble
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.2 Install Gazebo Classic
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-dev
```

### 1.3 Install additional Gazebo tools
```bash
sudo apt install gazebo libgazebo-dev
```

## Step 2: Install Unity (Unity Hub + Editor)

### 2.1 Install Unity Hub
For Linux systems, Unity Hub is not officially supported, so we'll use the Unity Editor directly or set up a headless environment for the examples. For learning purposes, we'll focus on the ROS integration aspects:

```bash
# Install prerequisites for Unity simulation
sudo apt update
sudo apt install ros-humble-rosbridge-suite ros-humble-tf2-web-republisher
```

### 2.2 For actual Unity development (optional):
- Download Unity Hub from Unity's website
- Install Unity 2022.3 LTS with the "Universal Render Pipeline" package
- Install the Unity Robotics Simulation Package via Package Manager

## Step 3: Set up ROS-Unity Bridge

### 3.1 Install ROS-TCP-Connector
```bash
# Clone the ROS-TCP-Connector
cd ~/ros2_fundamentals_ws/src
git clone -b ros2 https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

### 3.2 Build the workspace
```bash
cd ~/ros2_fundamentals_ws
colcon build
source install/setup.bash
```

## Step 4: Run Your First Simulation

### 4.1 Check Gazebo installation
```bash
gazebo --version
```

### 4.2 Launch a basic Gazebo world
```bash
# Terminal 1
source ~/ros2_fundamentals_ws/install/setup.bash
gazebo
```

### 4.3 Launch a sample robot simulation
```bash
# Terminal 2
source ~/ros2_fundamentals_ws/install/setup.bash
# Launch a simple robot model in Gazebo
ros2 launch gazebo_ros empty_world.launch.py world_name:=my_world.world
```

## Step 5: Create Your First Simulation Example

### 5.1 Create a simple world file
Create `~/ros2_fundamentals_ws/src/my_robot_simulation/worlds/basic-world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="basic_world">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -1</direction>
    </light>

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
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 5.2 Launch your custom world
```bash
gazebo ~/ros2_fundamentals_ws/src/my_robot_simulation/worlds/basic-world.world
```

## Step 6: Verify Your Setup

Run these commands to verify everything is working:

```bash
# Check Gazebo processes
ps aux | grep gazebo

# Check ROS 2 nodes (after launching simulation)
ros2 node list

# Check available Gazebo services
ros2 service list | grep /gazebo
```

## Next Steps

Now that you have a working digital twin simulation environment, continue with the full module:

1. **Chapter 1**: Complete the Physics Simulation with Gazebo to understand digital twin concepts
2. **Chapter 2**: Learn Sensor Simulation and data pipelines in ROS 2
3. **Chapter 3**: Explore Unity integration for high-fidelity human-robot interaction

## Troubleshooting

### Common Issues

**Issue**: "gazebo: command not found"
**Solution**: Ensure you've installed Gazebo and sourced your ROS 2 environment: `source /opt/ros/humble/setup.bash`

**Issue**: Gazebo GUI doesn't start properly
**Solution**: Check your graphics drivers and try running with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

**Issue**: ROS-TCP-Connector build fails
**Solution**: Make sure you have the correct ROS 2 distribution and all dependencies installed

**Issue**: Physics simulation is unstable
**Solution**: Adjust physics parameters in your world file (smaller step size, appropriate solver parameters)

## Getting Help

- Check the official Gazebo documentation: http://gazebosim.org/
- Visit the ROS answers forum: https://answers.ros.org/
- Unity Robotics documentation: https://github.com/Unity-Technologies/Unity-Robotics-Hub