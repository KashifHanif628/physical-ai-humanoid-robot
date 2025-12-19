---
title: "ROS 2 Package Development & Launch Management"
sidebar_label: "Week 3: ROS 2 Package Development"
description: "Learn about ROS 2 package development, launch file management, and workspace organization"
keywords: ["ros2", "packages", "development", "launch", "workspace", "cmake", "ament"]
---

# Week 3: ROS 2 Package Development & Launch Management

## Introduction

This week focuses on ROS 2 package development and launch file management, essential skills for creating well-structured Physical AI applications. You'll learn how to organize code into reusable packages, manage dependencies, and orchestrate complex robotic systems using launch files.

## ROS 2 Package Structure

A ROS 2 package is the basic building block of a ROS 2 system. It contains nodes, libraries, and other resources needed for a specific functionality.

### Basic Package Structure

```
physical_ai_package/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── src/                    # Source code
│   ├── publisher_node.cpp
│   └── subscriber_node.cpp
├── include/                # Header files
│   └── physical_ai/
├── launch/                 # Launch files
│   └── physical_ai_system.launch.py
├── config/                 # Configuration files
│   └── parameters.yaml
├── test/                   # Test files
└── scripts/                # Executable scripts
```

### package.xml: Package Manifest

The `package.xml` file contains metadata about the package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>physical_ai_examples</name>
  <version>1.0.0</version>
  <description>Physical AI examples for humanoid robotics</description>
  <maintainer email="student@physical-ai-humanoid-robot.edu">Student</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <exec_depend>ros2launch</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt: Build Configuration

For C++ packages, the `CMakeLists.txt` defines build instructions:

```cmake
cmake_minimum_required(VERSION 3.8)
project(physical_ai_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(physical_ai_publisher src/publisher_node.cpp)
ament_target_dependencies(physical_ai_publisher
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(physical_ai_subscriber src/subscriber_node.cpp)
ament_target_dependencies(physical_ai_subscriber
  rclcpp
  std_msgs
  sensor_msgs
)

# Install executables
install(TARGETS
  physical_ai_publisher
  physical_ai_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Python Package Structure

For Python-based Physical AI packages, the structure is slightly different:

```
physical_ai_py_examples/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/physical_ai_py_examples
├── physical_ai_py_examples/
│   ├── __init__.py
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   └── physical_ai_processor.py
└── test/
    └── test_physical_ai.py
```

### setup.py: Python Package Configuration

```python
from setuptools import setup
import os
from glob import glob

package_name = 'physical_ai_py_examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@physical-ai-humanoid-robot.edu',
    description='Physical AI examples for humanoid robotics (Python)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'physical_ai_publisher = physical_ai_py_examples.publisher_node:main',
            'physical_ai_subscriber = physical_ai_py_examples.subscriber_node:main',
            'physical_ai_processor = physical_ai_py_examples.physical_ai_processor:main',
        ],
    },
)
```

## Physical AI Package Example

Let's create a complete Physical AI package example:

### Physical AI Publisher Node

```python
#!/usr/bin/env python3

"""
Physical AI Publisher Node
Publishes sensor data and AI decisions for Physical AI systems
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import random
import math

class PhysicalAIPublisher(Node):
    def __init__(self):
        super().__init__('physical_ai_publisher')

        # Publisher for sensor data
        self.sensor_publisher = self.create_publisher(LaserScan, 'physical_ai/sensor_scan', 10)

        # Publisher for AI decisions
        self.ai_publisher = self.create_publisher(String, 'physical_ai/decisions', 10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for publishing sensor data
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

        # Timer for AI decisions
        self.ai_timer = self.create_timer(0.5, self.publish_ai_decision)  # 2 Hz

        # Timer for robot commands
        self.cmd_timer = self.create_timer(0.05, self.publish_robot_command)  # 20 Hz

        self.get_logger().info('Physical AI Publisher initialized')

        # Simulated robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Simulated environment
        self.obstacles = [(2.0, 1.0), (3.0, -1.0), (1.5, 2.0)]  # Fixed obstacles

    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Laser scan parameters
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 10.0

        # Calculate number of readings
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1

        # Simulate ranges based on robot position and obstacles
        ranges = []
        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment

            # Calculate distance to nearest obstacle in this direction
            min_distance = msg.range_max
            for obs_x, obs_y in self.obstacles:
                # Calculate relative position of obstacle
                rel_x = obs_x - self.robot_x
                rel_y = obs_y - self.robot_y

                # Calculate distance and angle to obstacle
                distance_to_obs = math.sqrt(rel_x**2 + rel_y**2)
                angle_to_obs = math.atan2(rel_y, rel_x) - self.robot_theta

                # Normalize angle to [-π, π]
                while angle_to_obs > math.pi:
                    angle_to_obs -= 2 * math.pi
                while angle_to_obs < -math.pi:
                    angle_to_obs += 2 * math.pi

                # If this direction aligns with obstacle direction, record distance
                if abs(angle - angle_to_obs) < 0.1:  # Within 0.1 radian (~5.7 degrees)
                    min_distance = min(min_distance, distance_to_obs - 0.2)  # Account for robot size

            # Add some noise to the sensor reading
            noisy_distance = min_distance + random.uniform(-0.05, 0.05)
            ranges.append(max(msg.range_min, min(noisy_distance, msg.range_max)))

        msg.ranges = ranges
        msg.intensities = [1.0] * len(ranges)  # All intensities same for simplicity

        self.sensor_publisher.publish(msg)

    def publish_ai_decision(self):
        """Publish AI decisions based on sensor data"""
        # Simple AI decision based on sensor data
        msg = String()

        # In a real system, this would be more complex AI processing
        # For simulation, we'll make simple decisions based on sensor data
        msg.data = self.make_simple_decision()

        self.ai_publisher.publish(msg)
        self.get_logger().info(f'Published AI decision: {msg.data}')

    def make_simple_decision(self):
        """Make a simple AI decision based on sensor data"""
        # For simulation purposes, return simple navigation decisions
        # based on proximity to obstacles

        # This is a simplified representation - in reality, you'd process
        # the actual sensor data published to the topic
        obstacle_distances = [random.uniform(0.5, 5.0) for _ in range(10)]
        closest_obstacle = min(obstacle_distances)

        if closest_obstacle < 1.0:
            return "AVOID_OBSTACLE_TURN_RIGHT"
        elif closest_obstacle < 2.0:
            return "SLOW_DOWN"
        else:
            return "MOVE_FORWARD_NORMAL_SPEED"

    def publish_robot_command(self):
        """Publish robot movement commands"""
        cmd_msg = Twist()

        # For simulation, use simple reactive behavior
        decision = self.make_simple_decision()

        if "AVOID" in decision:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn right
        elif "SLOW" in decision:
            cmd_msg.linear.x = 0.2  # Slow forward
            cmd_msg.angular.z = 0.0
        else:
            cmd_msg.linear.x = 0.5  # Normal speed forward
            cmd_msg.angular.z = 0.0

        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)

    publisher_node = PhysicalAIPublisher()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Physical AI Processing Node

```python
#!/usr/bin/env python3

"""
Physical AI Processing Node
Processes sensor data with AI algorithms and makes decisions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from collections import deque

class PhysicalAIProcessor(Node):
    def __init__(self):
        super().__init__('physical_ai_processor')

        # Subscription to sensor data
        self.sensor_subscription = self.create_subscription(
            LaserScan,
            'physical_ai/sensor_scan',
            self.sensor_callback,
            10
        )

        # Publisher for AI decisions
        self.decision_publisher = self.create_publisher(String, 'physical_ai/processed_decisions', 10)

        # Publisher for processed robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'physical_ai/cmd_vel', 10)

        self.get_logger().info('Physical AI Processor initialized')

        # Buffer for smoothing decisions
        self.decision_buffer = deque(maxlen=5)

        # AI model parameters (simplified for this example)
        self.obstacle_threshold = 1.0
        self.safety_margin = 0.5

    def sensor_callback(self, msg):
        """Process incoming sensor data with AI algorithms"""
        try:
            # Process the laser scan data
            processed_data = self.process_laser_scan(msg)

            # Apply AI decision making
            ai_decision = self.apply_ai_decision(processed_data)

            # Smooth the decision using buffer
            smoothed_decision = self.smooth_decision(ai_decision)

            # Publish AI decision
            decision_msg = String()
            decision_msg.data = smoothed_decision
            self.decision_publisher.publish(decision_msg)

            # Convert decision to robot command
            robot_cmd = self.convert_decision_to_command(smoothed_decision)
            self.cmd_publisher.publish(robot_cmd)

            self.get_logger().info(f'Processed decision: {smoothed_decision}')

        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {e}')

    def process_laser_scan(self, scan_msg):
        """Process laser scan data with AI techniques"""
        # Convert ranges to numpy array for easier processing
        ranges = np.array(scan_msg.ranges)

        # Filter out invalid ranges (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Calculate statistics
        if len(valid_ranges) > 0:
            min_range = np.min(valid_ranges)
            mean_range = np.mean(valid_ranges)
            std_range = np.std(valid_ranges)
        else:
            min_range = float('inf')
            mean_range = float('inf')
            std_range = 0.0

        # Calculate obstacle density in different sectors
        sector_size = len(ranges) // 3  # Divide into 3 sectors: left, front, right
        left_sector = ranges[:sector_size]
        front_sector = ranges[sector_size:2*sector_size]
        right_sector = ranges[2*sector_size:]

        left_obstacles = np.sum((left_sector < self.obstacle_threshold) & np.isfinite(left_sector))
        front_obstacles = np.sum((front_sector < self.obstacle_threshold) & np.isfinite(front_sector))
        right_obstacles = np.sum((right_sector < self.obstacle_threshold) & np.isfinite(right_sector))

        return {
            'min_range': min_range,
            'mean_range': mean_range,
            'std_range': std_range,
            'left_obstacles': left_obstacles,
            'front_obstacles': front_obstacles,
            'right_obstacles': right_obstacles,
            'valid_readings': len(valid_ranges),
            'total_readings': len(ranges)
        }

    def apply_ai_decision(self, processed_data):
        """Apply AI decision making based on processed sensor data"""
        # This is a simplified AI decision process
        # In a real system, this would use more sophisticated ML/AI algorithms

        min_range = processed_data['min_range']
        front_obstacles = processed_data['front_obstacles']
        left_obstacles = processed_data['left_obstacles']
        right_obstacles = processed_data['right_obstacles']

        # Decision logic based on obstacle detection
        if min_range < self.obstacle_threshold:
            if front_obstacles > 0:
                # Obstacles in front - decide turn direction based on left/right comparison
                if left_obstacles <= right_obstacles:
                    return "TURN_LEFT_TO_AVOID"
                else:
                    return "TURN_RIGHT_TO_AVOID"
            elif left_obstacles > 0 and right_obstacles > 0:
                # Obstacles on both sides - turn toward clearer direction
                if left_obstacles <= right_obstacles:
                    return "PREFER_LEFT_PATH"
                else:
                    return "PREFER_RIGHT_PATH"
            else:
                # Close obstacle but not in critical directions
                return "SLOW_DOWN_CAUTIOUSLY"
        else:
            # Path appears clear
            return "PROCEED_NORMALLY"

    def smooth_decision(self, new_decision):
        """Smooth AI decisions over time to reduce erratic behavior"""
        self.decision_buffer.append(new_decision)

        # For simple smoothing, return the most common decision in buffer
        if len(self.decision_buffer) > 0:
            # Count occurrences of each decision
            decision_counts = {}
            for decision in self.decision_buffer:
                decision_counts[decision] = decision_counts.get(decision, 0) + 1

            # Return the most frequent decision
            return max(decision_counts, key=decision_counts.get)

        return new_decision

    def convert_decision_to_command(self, decision):
        """Convert AI decision to robot command"""
        cmd_msg = Twist()

        if "TURN_LEFT" in decision:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.4  # Turn left
        elif "TURN_RIGHT" in decision:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = -0.4  # Turn right
        elif "PREFER_LEFT" in decision:
            cmd_msg.linear.x = 0.3  # Move forward slowly
            cmd_msg.angular.z = 0.1  # Slight left bias
        elif "PREFER_RIGHT" in decision:
            cmd_msg.linear.x = 0.3  # Move forward slowly
            cmd_msg.angular.z = -0.1  # Slight right bias
        elif "SLOW" in decision:
            cmd_msg.linear.x = 0.2  # Move slowly
            cmd_msg.angular.z = 0.0
        else:  # "PROCEED" or default
            cmd_msg.linear.x = 0.5  # Normal forward speed
            cmd_msg.angular.z = 0.0

        return cmd_msg

def main(args=None):
    rclpy.init(args=args)

    processor_node = PhysicalAIProcessor()

    try:
        rclpy.spin(processor_node)
    except KeyboardInterrupt:
        pass
    finally:
        processor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File Management

Launch files orchestrate multiple nodes and configure parameters for complex Physical AI systems.

### Basic Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for nodes'
    )

    # Physical AI publisher node
    physical_ai_publisher = Node(
        package='physical_ai_py_examples',
        executable='physical_ai_publisher',
        name='physical_ai_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/physical_ai/sensor_scan', '/sensors/laser_scan'),
            ('/physical_ai/decisions', '/ai/raw_decisions'),
        ],
        arguments=['--log-level', log_level],
        output='screen'
    )

    # Physical AI processing node
    physical_ai_processor = Node(
        package='physical_ai_py_examples',
        executable='physical_ai_processor',
        name='physical_ai_processor',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'obstacle_threshold': 1.0},
            {'safety_margin': 0.5},
        ],
        remappings=[
            ('/physical_ai/sensor_scan', '/sensors/laser_scan'),
            ('/physical_ai/processed_decisions', '/ai/processed_decisions'),
            ('/physical_ai/cmd_vel', '/cmd_vel'),
        ],
        arguments=['--log-level', log_level],
        output='screen'
    )

    # Robot simulator node (if needed)
    robot_simulator = Node(
        package='physical_ai_py_examples',
        executable='robot_simulator',
        name='robot_simulator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'update_rate': 50.0},
        ],
        arguments=['--log-level', log_level],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_log_level_arg)

    # Add nodes
    ld.add_action(physical_ai_publisher)
    ld.add_action(physical_ai_processor)
    ld.add_action(robot_simulator)

    return ld
```

### Advanced Launch File with Conditional Logic

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_logging = LaunchConfiguration('enable_logging')
    ai_model_type = LaunchConfiguration('ai_model_type')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable detailed logging'
    )

    declare_ai_model_type_arg = DeclareLaunchArgument(
        'ai_model_type',
        default_value='basic',
        choices=['basic', 'advanced', 'reinforcement_learning'],
        description='Type of AI model to use'
    )

    # Define conditional groups based on AI model type
    basic_ai_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", ai_model_type, "' == 'basic'"])
        ),
        actions=[
            Node(
                package='physical_ai_py_examples',
                executable='physical_ai_basic_processor',
                name='physical_ai_basic_processor',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    advanced_ai_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", ai_model_type, "' == 'advanced'"])
        ),
        actions=[
            Node(
                package='physical_ai_py_examples',
                executable='physical_ai_advanced_processor',
                name='physical_ai_advanced_processor',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    rl_ai_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", ai_model_type, "' == 'reinforcement_learning'"])
        ),
        actions=[
            Node(
                package='physical_ai_py_examples',
                executable='physical_ai_rl_processor',
                name='physical_ai_rl_processor',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # Common nodes for all configurations
    common_nodes = [
        Node(
            package='physical_ai_py_examples',
            executable='physical_ai_publisher',
            name='physical_ai_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='physical_ai_py_examples',
            executable='physical_ai_monitor',
            name='physical_ai_monitor',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'enable_logging': enable_logging}
            ],
            output='screen'
        )
    ]

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_enable_logging_arg)
    ld.add_action(declare_ai_model_type_arg)

    # Add common nodes
    for node in common_nodes:
        ld.add_action(node)

    # Add conditional AI groups
    ld.add_action(basic_ai_group)
    ld.add_action(advanced_ai_group)
    ld.add_action(rl_ai_group)

    return ld
```

## Configuration Management

Configuration files allow you to parameterize your Physical AI system without recompilation.

### Parameters YAML File

```yaml
physical_ai_publisher:
  ros__parameters:
    use_sim_time: false
    sensor_update_rate: 10.0
    obstacle_threshold: 1.0
    safety_margin: 0.5
    robot_speed: 0.5
    turning_speed: 0.4

physical_ai_processor:
  ros__parameters:
    use_sim_time: false
    ai_algorithm: "basic_reactive"
    obstacle_threshold: 1.0
    safety_margin: 0.5
    decision_smoothing_factor: 0.7
    min_obstacle_distance: 0.3
    max_robot_speed: 1.0
    sensor_noise_threshold: 0.1

robot_simulator:
  ros__parameters:
    use_sim_time: false
    physics_update_rate: 100.0
    gravity: -9.81
    robot_mass: 50.0
    friction_coefficient: 0.1
```

## Best Practices for Physical AI Package Development

### 1. Modular Design
- Separate perception, decision-making, and action components
- Use composition over inheritance
- Keep nodes focused on single responsibilities

### 2. Performance Optimization
- Use appropriate message frequencies
- Implement efficient data structures
- Consider computational complexity of AI algorithms

### 3. Safety Considerations
- Implement safety state machines
- Use watchdog timers for critical systems
- Validate all inputs and outputs

### 4. Testing Strategy
- Unit tests for individual components
- Integration tests for complete systems
- Simulation-based validation before physical deployment

## Learning Objectives

By the end of Week 3, you should be able to:
1. Create well-structured ROS 2 packages for Physical AI applications
2. Implement launch files for orchestrating complex systems
3. Manage configuration parameters effectively
4. Apply best practices for Physical AI package development

## Exercises

### Exercise 1: Basic Package Creation (Beginner)
- **Time**: 60 minutes
- **Objective**: Create a simple ROS 2 package with one node
- **Steps**: Create a package with proper structure, build files, and a basic publisher node
- **Expected Outcome**: Working ROS 2 package that builds and runs successfully

### Exercise 2: Multi-Node Package (Intermediate)
- **Time**: 90 minutes
- **Objective**: Create a package with multiple interacting nodes
- **Steps**: Create a package with publisher, subscriber, and service nodes that work together
- **Expected Outcome**: Integrated multi-node system with proper communication patterns

### Exercise 3: Launch File Configuration (Advanced)
- **Time**: 120 minutes
- **Objective**: Create a comprehensive launch file with parameters and conditional logic
- **Steps**: Design a launch file that starts multiple nodes with configuration options
- **Expected Outcome**: Flexible launch system that can handle different operational modes

## Summary

Week 3 covered ROS 2 package development and launch management, essential skills for creating well-structured Physical AI applications. You learned about package structure, build configuration, launch files, and configuration management. In Week 4, we'll explore robot simulation with Gazebo and Unity visualization.