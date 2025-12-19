---
title: "ROS 2 Architecture & Communication Patterns"
sidebar_label: "Week 2: ROS 2 Architecture"
description: "Learn about ROS 2 architecture, nodes, topics, services, and actions"
keywords: ["ros2", "architecture", "nodes", "topics", "services", "actions", "communication"]
---

# Week 2: ROS 2 Architecture & Communication Patterns

## Introduction

This week introduces the Robot Operating System (ROS 2) architecture, which forms the backbone of most modern robotics applications. ROS 2 provides the communication infrastructure that enables different components of a robotic system to work together. Understanding ROS 2 architecture is crucial for implementing Physical AI systems that coordinate perception, decision-making, and action.

## ROS 2 Fundamentals

ROS 2 is a flexible framework for writing robot software. It's a collection of libraries and tools that help you build robot applications. The key concepts include:

### Nodes
Nodes are processes that perform computation. In ROS 2, nodes are written in various programming languages and can run on different machines. Each node can perform a specific function in the robot system.

### Communication Primitives
ROS 2 provides several ways for nodes to communicate:
- **Topics**: Asynchronous message passing for data streams
- **Services**: Synchronous request/response communication
- **Actions**: Goal-oriented communication with feedback and status

## Nodes: The Building Blocks

A node is the fundamental unit of computation in ROS 2. Each node performs a specific task and communicates with other nodes through topics, services, or actions.

### Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PhysicalAIRobotNode(Node):
    def __init__(self):
        super().__init__('physical_ai_robot')

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'ai_commands',
            self.command_callback,
            10
        )

        # Create timer for periodic publishing
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('Physical AI Robot Node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = 'Physical AI Robot: Operational'
        self.publisher.publish(msg)

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process AI decision and execute physical action
        self.execute_robot_action(msg.data)

    def execute_robot_action(self, command):
        # Translate AI decision to physical action
        if 'move' in command:
            self.get_logger().info('Executing movement command')
        elif 'stop' in command:
            self.get_logger().info('Stopping robot')

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAIRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Asynchronous Communication

Topics enable asynchronous communication between nodes. Publishers send messages to a topic, and subscribers receive messages from that topic.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class PhysicalAISensorPublisher(Node):
    def __init__(self):
        super().__init__('physical_ai_sensor_publisher')
        self.publisher = self.create_publisher(LaserScan, 'physical_ai/laser_scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Simulate laser scan data
        msg.angle_min = -1.57  # -90 degrees
        msg.angle_max = 1.57   # 90 degrees
        msg.angle_increment = 0.01
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate random ranges (simulated sensor data)
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = [random.uniform(0.5, 5.0) for _ in range(num_readings)]

        self.publisher.publish(msg)
        self.get_logger().info(f'Published laser scan with {len(msg.ranges)} readings')

def main(args=None):
    rclpy.init(args=args)
    publisher = PhysicalAISensorPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PhysicalAIProcessor(Node):
    def __init__(self):
        super().__init__('physical_ai_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'physical_ai/laser_scan',
            self.scan_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Physical AI Processor initialized')

    def scan_callback(self, msg):
        # Process laser scan data with AI logic
        obstacle_detected = self.detect_obstacles(msg)

        if obstacle_detected:
            self.get_logger().info('Obstacle detected! Planning avoidance maneuver.')
            self.avoid_obstacle()
        else:
            self.get_logger().info('Path clear. Continuing forward.')
            self.move_forward()

    def detect_obstacles(self, scan_msg):
        # Simple obstacle detection using AI-like logic
        min_distance = min(scan_msg.ranges)
        return min_distance < 1.0  # Obstacle within 1 meter

    def avoid_obstacle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Turn right
        self.cmd_publisher.publish(cmd)

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        cmd.angular.z = 0.0
        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    processor = PhysicalAIProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()
```

## Services: Synchronous Communication

Services provide synchronous request/response communication. A client sends a request and waits for a response from a server.

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class PhysicalAIService(Node):
    def __init__(self):
        super().__init__('physical_ai_service')

        # Create service with custom callback group for concurrency
        self.service = self.create_service(
            SetBool,
            'physical_ai/emergency_stop',
            self.emergency_stop_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.is_operational = True
        self.get_logger().info('Physical AI Emergency Stop Service ready')

    def emergency_stop_callback(self, request, response):
        if request.data:  # Emergency stop requested
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.is_operational = False
            response.success = True
            response.message = 'Emergency stop activated'
        else:  # Resume operation
            self.get_logger().info('Operation resumed')
            self.is_operational = True
            response.success = True
            response.message = 'Operation resumed'

        return response

def main(args=None):
    rclpy.init(args=args)
    service = PhysicalAIService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()
```

### Service Client Example

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ServiceClientExample(Node):
    def __init__(self):
        super().__init__('service_client_example')
        self.cli = self.create_client(SetBool, 'physical_ai/emergency_stop')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetBool.Request()

    def send_request(self, stop_request):
        self.req.data = stop_request
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = ServiceClientExample()

    # Request emergency stop
    response = client.send_request(True)
    if response:
        print(f'Response: {response.message}')

    # Resume operation
    response = client.send_request(False)
    if response:
        print(f'Response: {response.message}')

    client.destroy_node()
    rclpy.shutdown()
```

## Actions: Goal-Oriented Communication

Actions are used for long-running tasks that provide feedback and status updates. They're ideal for navigation, manipulation, and other complex behaviors.

### Action Server Example

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from example_interfaces.action import Fibonacci

class PhysicalAIActionServer(Node):
    def __init__(self):
        super().__init__('physical_ai_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'physical_ai/navigation_action',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = PhysicalAIActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy()
        rclpy.shutdown()
```

## Launch Files: Orchestrating Multiple Nodes

Launch files allow you to start multiple nodes with a single command and configure their parameters.

### Example Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Physical AI sensor publisher
        Node(
            package='physical_ai_examples',
            executable='sensor_publisher',
            name='physical_ai_sensor_publisher',
            parameters=[
                {'sensor_range': 10.0},
                {'update_rate': 10.0}
            ],
            remappings=[
                ('/scan', '/physical_ai/scan')
            ]
        ),

        # Physical AI processor
        Node(
            package='physical_ai_examples',
            executable='processor',
            name='physical_ai_processor',
            parameters=[
                {'obstacle_threshold': 1.0},
                {'safety_margin': 0.5}
            ],
            remappings=[
                ('/cmd_vel', '/physical_ai/cmd_vel')
            ]
        ),

        # Physical AI service
        Node(
            package='physical_ai_examples',
            executable='service',
            name='physical_ai_service',
            parameters=[
                {'emergency_response_time': 0.1}
            ]
        )
    ])
```

## Best Practices for ROS 2 Architecture

### Design Principles
1. **Single Responsibility**: Each node should have a single, well-defined purpose
2. **Loose Coupling**: Minimize dependencies between nodes
3. **High Cohesion**: Group related functionality within nodes
4. **Clear Interfaces**: Use well-defined message types and service interfaces

### Performance Considerations
- Use appropriate QoS settings for different types of data
- Implement proper error handling and recovery
- Consider network topology for distributed systems
- Optimize message frequency for real-time requirements

### Safety Considerations
- Implement emergency stop mechanisms
- Use proper safety state machines
- Validate all inputs and outputs
- Include comprehensive logging

## Learning Objectives

By the end of Week 2, you should be able to:
1. Create and configure ROS 2 nodes for Physical AI applications
2. Implement different communication patterns (topics, services, actions)
3. Design launch files for orchestrating multiple nodes
4. Apply best practices for ROS 2 architecture in Physical AI systems

## Exercises

### Exercise 1: Basic Node Implementation (Beginner)
- **Time**: 45 minutes
- **Objective**: Create a simple ROS 2 node that publishes sensor data
- **Steps**: Implement a node that publishes simulated sensor data with proper ROS 2 structure
- **Expected Outcome**: Working ROS 2 node that publishes messages to a topic

### Exercise 2: Communication Pattern Integration (Intermediate)
- **Time**: 60 minutes
- **Objective**: Create a system with publisher, subscriber, and service
- **Steps**: Implement a Physical AI system that uses all three communication patterns
- **Expected Outcome**: Integrated system with proper communication between components

### Exercise 3: Launch File Configuration (Advanced)
- **Time**: 75 minutes
- **Objective**: Create a launch file for a complete Physical AI system
- **Steps**: Design a launch file that starts multiple nodes with proper configuration
- **Expected Outcome**: Launch file that starts a complete Physical AI application

## Summary

Week 2 has covered the essential ROS 2 architecture concepts that form the foundation for implementing Physical AI systems. You now understand nodes, topics, services, actions, and launch files. In Week 3, we'll continue with more advanced ROS 2 concepts and package development.