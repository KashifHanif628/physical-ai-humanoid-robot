---
title: "Gazebo Simulation Setup & URDF/SDF Formats"
sidebar_label: "Week 4: Gazebo Simulation"
description: "Learn about Gazebo setup, URDF/SDF formats, physics and sensor simulation"
keywords: ["gazebo", "simulation", "urdf", "sdf", "physics", "sensors", "robotics"]
---

# Week 4: Gazebo Simulation Setup & URDF/SDF Formats

## Introduction

This week focuses on robot simulation using Gazebo, a powerful physics-based simulation environment that's essential for Physical AI development. You'll learn about URDF (Unified Robot Description Format) and SDF (Simulation Description Format) for modeling robots and environments, physics simulation, and sensor modeling. Simulation provides a safe and cost-effective way to test Physical AI algorithms before deploying on physical robots.

## Gazebo Simulation Fundamentals

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. For Physical AI development, Gazebo serves as a bridge between digital AI systems and physical robot behavior.

### Key Features of Gazebo
- **Physics Simulation**: Accurate simulation of rigid body dynamics, collisions, and contact forces
- **Sensor Simulation**: Realistic simulation of cameras, LiDAR, IMU, GPS, and other sensors
- **Graphics**: High-quality rendering with support for realistic lighting and materials
- **Plugins**: Extensible architecture for custom simulation capabilities
- **ROS Integration**: Native support for ROS/ROS 2 communication

### Installing Gazebo

For ROS 2 Humble, install Gazebo Garden:

```bash
# Add Gazebo repository
sudo curl -sSL https://get.gazebosim.org | sh

# Install Gazebo Garden
sudo apt-get install gz-garden

# Install ROS 2 Gazebo packages
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-dev
```

## URDF: Unified Robot Description Format

URDF is the standard format for representing robot models in ROS. It describes the robot's physical and kinematic properties using XML.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="physical_ai_humanoid">
  <!-- Robot base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

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
  </link>

  <!-- Robot leg link -->
  <link name="leg_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>

    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.3"/>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.3"/>
    </collision>
  </link>

  <!-- Joint connecting base to leg -->
  <joint name="base_to_leg" type="fixed">
    <parent link="base_link"/>
    <child link="leg_link"/>
    <origin xyz="0 0 -0.5"/>
  </joint>
</robot>
```

### Advanced URDF Features

#### Transmission Elements
Transmission elements define how actuators connect to joints:

```xml
<transmission name="wheel_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

#### Gazebo Plugins
Gazebo-specific plugins can be included in URDF:

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/physical_ai_robot</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <update_rate>30</update_rate>
    <left_joint>wheel_left_joint</left_joint>
    <right_joint>wheel_right_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>
</gazebo>
```

## SDF: Simulation Description Format

SDF is Gazebo's native simulation format that provides more features than URDF, particularly for simulation-specific elements.

### Basic SDF Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physical_ai_world">
    <!-- Physics engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include models from Gazebo Fuel -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Spawn our robot -->
    <include>
      <uri>model://physical_ai_humanoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Custom objects in the environment -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="table_link">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sensors in the environment -->
    <model name="environment_sensors">
      <pose>0 0 2 0 0 0</pose>
      <link name="sensor_link">
        <sensor name="overhead_camera" type="camera">
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>10</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
```

## Physics Simulation in Gazebo

Gazebo provides accurate physics simulation using various physics engines. The choice of physics engine and parameters significantly affects simulation accuracy and performance.

### Physics Engine Configuration

```xml
<physics type="ode" name="default_physics">
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>1000</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

### Key Physics Parameters

- **Gravity**: Defines gravitational acceleration (typically 9.8 m/s² downward)
- **Max Step Size**: Time step for physics simulation (smaller = more accurate but slower)
- **Real Time Factor**: Ratio of simulation time to real time (1.0 = real-time)
- **Solver Iterations**: Number of iterations for constraint solving (more = more stable but slower)

## Sensor Simulation

Accurate sensor simulation is crucial for Physical AI development. Gazebo provides realistic simulation of various sensor types.

### Camera Sensor

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
  <topic>camera/image_raw</topic>
</sensor>
```

### LiDAR Sensor

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
  <topic>scan</topic>
</sensor>
```

### IMU Sensor

```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <visualize>false</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.17</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.17</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.17</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Physical AI Simulation Integration

For Physical AI applications, we need to connect our AI systems to the simulated robot. This involves setting up proper interfaces between the AI decision-making system and the simulated robot.

### Simulation Launch File

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Choose one of: empty, warehouse, maze'
    )

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('physical_ai_simulation'),
                'worlds',
                [world, '.world']
            ]),
            'use_sim_time': use_sim_time
        }.items()
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('physical_ai_simulation'),
                    'urdf',
                    'physical_ai_humanoid.urdf.xacro'
                ])
            ])
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'physical_ai_humanoid',
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )

    # Physical AI processor node
    physical_ai_processor = Node(
        package='physical_ai_py_examples',
        executable='physical_ai_processor',
        name='physical_ai_processor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_world_arg)

    # Add actions
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(physical_ai_processor)

    return ld
```

## Creating a Physical AI Humanoid Robot Model

Let's create a complete humanoid robot model for simulation:

### Physical AI Humanoid URDF (Xacro Format)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="physical_ai_humanoid">

  <!-- Constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_mass" value="5.0" />
  <xacro:property name="base_radius" value="0.15" />
  <xacro:property name="base_length" value="0.3" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.4235294117647059 0.0392156862745098 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.8705882352941177 0.8117647058823529 0.7647058823529411 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="${base_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_length}"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 ${base_length/2 + 0.1}" rpy="0 0 0"/>
  </joint>

  <link name="head_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Sensors in head -->
  <gazebo reference="head_link">
    <sensor name="camera" type="camera">
      <pose>0.1 0 0 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <topic>camera/image_raw</topic>
    </sensor>
  </gazebo>

  <!-- Left Arm -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm_link"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_upper_arm_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Forearm -->
  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm_link"/>
    <child link="left_forearm_link"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_forearm_link">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.15"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh_link"/>
    <origin xyz="0 -0.1 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_thigh_link">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Left Shin -->
  <joint name="left_knee" type="revolute">
    <parent link="left_thigh_link"/>
    <child link="left_shin_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_shin_link">
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.0015" ixy="0" ixz="0" iyy="0.0015" iyz="0" izz="0.0015"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Right Leg (symmetric) -->
  <joint name="right_hip" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh_link"/>
    <origin xyz="0 0.1 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_thigh_link">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh_link"/>
    <child link="right_shin_link"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="0" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_shin_link">
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.0015" ixy="0" ixz="0" iyy="0.0015" iyz="0" izz="0.0015"/>
    </inertial>

    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>

    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <parameters>$(find physical_ai_simulation)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head_link">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_upper_arm_link">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="left_forearm_link">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="left_thigh_link">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="left_shin_link">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_thigh_link">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="right_shin_link">
    <material>Gazebo/Green</material>
  </gazebo>

</robot>
```

## Simulation Best Practices for Physical AI

### 1. Realistic Physics Parameters
- Use accurate mass, inertia, and friction values for your robot
- Match simulation parameters to physical robot characteristics
- Validate simulation behavior against physical robot when possible

### 2. Sensor Accuracy
- Include realistic noise models in sensor simulations
- Match sensor specifications to physical hardware
- Validate sensor data quality and range limitations

### 3. Performance Optimization
- Use appropriate collision geometries (simpler than visual meshes)
- Limit simulation update rates to realistic sensor frequencies
- Use level-of-detail (LOD) models for complex environments

### 4. Sim-to-Real Transfer
- Document differences between simulation and reality
- Implement domain randomization to improve transfer
- Validate performance on physical robots when possible

## Learning Objectives

By the end of Week 4, you should be able to:
1. Create and configure robot models using URDF/Xacro
2. Set up Gazebo simulation environments with proper physics parameters
3. Simulate various sensor types with realistic noise models
4. Integrate Physical AI systems with simulated robots
5. Apply best practices for simulation accuracy and performance

## Exercises

### Exercise 1: Basic Robot Model (Beginner)
- **Time**: 45 minutes
- **Objective**: Create a simple wheeled robot model in URDF
- **Steps**: Design a basic robot with chassis, wheels, and sensors
- **Expected Outcome**: Working URDF model that spawns correctly in Gazebo

### Exercise 2: Humanoid Robot Model (Intermediate)
- **Time**: 90 minutes
- **Objective**: Create a humanoid robot model with articulated joints
- **Steps**: Design a humanoid model with arms, legs, and head with sensors
- **Expected Outcome**: Complete humanoid URDF model with proper kinematics

### Exercise 3: Simulation Integration (Advanced)
- **Time**: 120 minutes
- **Objective**: Integrate Physical AI system with simulated robot
- **Steps**: Connect AI decision-making system to simulated robot with proper interfaces
- **Expected Outcome**: AI system controlling simulated robot in Gazebo environment

## Summary

Week 4 covered Gazebo simulation setup and URDF/SDF formats, essential for Physical AI development. You learned to create robot models, configure physics parameters, simulate sensors, and integrate AI systems with simulated robots. In Week 5, we'll explore Unity visualization and simulation for enhanced human-robot interaction design.