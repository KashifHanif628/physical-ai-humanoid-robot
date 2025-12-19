---
title: Python Control with rclpy
sidebar_label: Chapter 2: Python Control
description: Learn how to create ROS 2 nodes in Python using the rclpy library
keywords: ros2, python, rclpy, nodes, control, robotics
---

# Python Control with rclpy

## Learning Objectives

By the end of this chapter, you will be able to:
- Create ROS 2 nodes in Python using the rclpy library
- Implement publishers, subscribers, services, and actions
- Understand node lifecycle management
- Apply error handling best practices
- Work with parameters in ROS 2 nodes

## Prerequisites

Before starting this chapter, you should:
- Have completed Chapter 1 (ROS 2 Installation and Basics)
- Have a working ROS 2 environment
- Understand basic ROS 2 concepts (nodes, topics, services)

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API that enables Python programs to communicate with other ROS 2 nodes. rclpy provides functionality such as:

- Creating and managing nodes
- Publishing and subscribing to topics
- Providing and using services
- Creating and managing actions
- Working with parameters

## Creating Your First Python Node

Let's start by creating a simple Python node that publishes a message to a topic.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

## Publishers and Subscribers

### Publishers

A publisher sends messages to a topic. Here's a more detailed example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherExample(Node):

    def __init__(self):
        super().__init__('publisher_example')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello from Python: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.i += 1
```

### Subscribers

A subscriber receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberExample(Node):

    def __init__(self):
        super().__init__('subscriber_example')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Services

Services provide a request/response communication pattern. Here's how to create a service server:

### Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions

Actions are used for long-running tasks with feedback. Here's an example:

### Action Server

```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result
```

## Parameters

Parameters allow you to configure your nodes. Here's how to work with them:

```python
import rclpy
from rclpy.node import Node

class ParameterExample(Node):

    def __init__(self):
        super().__init__('parameter_example')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('count_threshold', 10)

        # Get parameter values
        my_param = self.get_parameter('my_parameter').value
        threshold = self.get_parameter('count_threshold').value

        self.get_logger().info(f'My parameter: {my_param}')
        self.get_logger().info(f'Threshold: {threshold}')
```

## Error Handling and Best Practices

### Proper Node Lifecycle Management

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobustNode(Node):

    def __init__(self):
        super().__init__('robust_node')
        self.publisher_ = self.create_publisher(String, 'robust_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            msg = String()
            msg.data = 'Robust message'
            self.publisher_.publish(msg)
            self.get_logger().info('Published message successfully')
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobustNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Node Creation

### Node Composition

You can compose multiple nodes together in a single process:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'composed_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Composed message {self.i}'
        self.publisher.publish(msg)
        self.i += 1

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'composed_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Composed subscriber heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()
    subscriber_node = SubscriberNode()

    executor = MultiThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()
```

## Error Handling Best Practices

### Exception Handling in Callbacks

Always wrap your callback logic in try-catch blocks to prevent node crashes:

```python
def robust_callback(self, msg):
    try:
        # Process the message
        result = self.process_message(msg)
        self.publish_result(result)
    except ValueError as e:
        self.get_logger().error(f'Value error in callback: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error in callback: {e}')
```

### Parameter Validation

Validate parameters when your node starts:

```python
def __init__(self):
    super().__init__('validated_node')

    # Declare parameters
    self.declare_parameter('frequency', 1.0)
    self.declare_parameter('topic_name', 'default_topic')

    # Validate parameters
    frequency = self.get_parameter('frequency').value
    if frequency <= 0:
        self.get_logger().fatal('Frequency must be positive')
        raise ValueError('Invalid frequency parameter')

    topic_name = self.get_parameter('topic_name').value
    if not topic_name or not isinstance(topic_name, str):
        self.get_logger().fatal('Invalid topic name')
        raise ValueError('Invalid topic name parameter')
```

## Node Lifecycle Management

### Proper Resource Cleanup

Always clean up resources in the destroy_node method:

```python
class LifecycleNode(Node):

    def __init__(self):
        super().__init__('lifecycle_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.file_handle = open('/tmp/my_file.txt', 'w')

    def timer_callback(self):
        self.file_handle.write('Timer tick\n')

    def destroy_node(self):
        if hasattr(self, 'file_handle') and self.file_handle:
            self.file_handle.close()
        if hasattr(self, 'timer'):
            self.timer.destroy()
        return super().destroy_node()
```

## Summary

In this chapter, you've learned:
- How to create ROS 2 nodes in Python using rclpy
- How to implement publishers, subscribers, services, and actions
- How to work with parameters in ROS 2 nodes
- Best practices for error handling and node lifecycle management
- Advanced concepts like node composition and parameter validation

In the next chapter, we'll explore humanoid robot URDF models and how to work with them.