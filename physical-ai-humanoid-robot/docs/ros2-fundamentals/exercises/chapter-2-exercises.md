---
title: Chapter 2 Exercises - Python Control with rclpy
sidebar_label: "Chapter 2 Exercises: Python Control with rclpy"
description: Exercises to practice creating ROS 2 nodes in Python
keywords: [ros2, python, exercises, rclpy, nodes, publishers, subscribers]
---

# Chapter 2 Exercises - Python Control with rclpy

## Exercise 1: Creating a Custom Publisher Node

**Difficulty**: Intermediate
**Estimated Time**: 30 minutes
**Prerequisites**: Understanding of basic ROS 2 concepts and Python

### Objective
Create a custom publisher node that publishes sensor data at a specific rate.

### Steps
1. Create a new Python file called `sensor_publisher.py`
2. Create a node called `sensor_publisher`
3. Set up a publisher that publishes `std_msgs/String` messages to a topic called `sensor_data`
4. Create a timer that publishes a message every 2 seconds
5. In each message, include a timestamp and a simulated sensor reading
6. Add logging to show when messages are published
7. Run the node and verify it's publishing messages

### Expected Outcome
- A working publisher node that publishes sensor data every 2 seconds
- Messages should include timestamp and simulated sensor reading
- Node should log each publication to the console

### Hints
- Import `std_msgs.msg.String` for the message type
- Use `time.time()` or `datetime` for timestamps
- Remember to call `rclpy.init()` and `rclpy.spin()`

---

## Exercise 2: Creating a Subscriber Node

**Difficulty**: Intermediate
**Estimated Time**: 25 minutes
**Prerequisites**: Completed Exercise 1 or have a publisher running

### Objective
Create a subscriber node that receives and processes messages from the sensor publisher.

### Steps
1. Create a new Python file called `sensor_subscriber.py`
2. Create a node called `sensor_subscriber`
3. Set up a subscription to the `sensor_data` topic
4. Create a callback function that processes incoming messages
5. In the callback, parse the message and log the sensor reading
6. Calculate and log the time difference between consecutive messages
7. Run the subscriber node while the publisher is running

### Expected Outcome
- A working subscriber node that receives messages from the publisher
- Console logs showing received sensor data and time differences
- Both nodes should work together without errors

### Hints
- Use `create_subscription()` to set up the subscription
- The callback function should accept the message as a parameter
- Store the previous timestamp to calculate time differences

---

## Exercise 3: Creating a Service Server and Client

**Difficulty**: Advanced
**Estimated Time**: 45 minutes
**Prerequisites**: Understanding of ROS 2 services

### Objective
Create a service that performs calculations on sensor data.

### Steps
1. Create a service server called `data_processor` that:
   - Provides a service called `process_sensor_data`
   - Accepts two sensor readings as parameters
   - Returns the average of the two readings
2. Create a service client that:
   - Calls the `process_sensor_data` service
   - Sends two sensor values
   - Receives and displays the result
3. Test the service by running both server and client

### Expected Outcome
- A working service server that calculates averages
- A service client that successfully calls the service
- Correct results returned from the service

### Hints
- Use `example_interfaces/srv/AddTwoInts` as a reference for service structure
- You'll need to create a custom service interface if needed, or use existing ones
- Service servers use `create_service()` and clients use `create_client()`

---

## Exercise 4: Working with Parameters

**Difficulty**: Intermediate
**Estimated Time**: 35 minutes
**Prerequisites**: Understanding of ROS 2 nodes

### Objective
Create a node that uses parameters to configure its behavior.

### Steps
1. Create a node called `configurable_publisher`
2. Declare the following parameters with default values:
   - `publish_rate` (default: 1.0 seconds)
   - `message_prefix` (default: "Configurable")
   - `max_count` (default: 10)
3. Use these parameters to control the node's behavior:
   - Adjust the timer period based on `publish_rate`
   - Use `message_prefix` in published messages
   - Stop publishing after `max_count` messages
4. Test the node with different parameter values

### Expected Outcome
- A node that behaves differently based on parameters
- Ability to change behavior by setting parameters
- Proper parameter declaration and usage

### Hints
- Use `declare_parameter()` to declare parameters
- Use `get_parameter()` to retrieve parameter values
- You can set parameters when running the node with `--ros-args -p parameter_name:=value`

---

## Exercise 5: Advanced Node with Proper Lifecycle Management

**Difficulty**: Advanced
**Estimated Time**: 40 minutes
**Prerequisites**: Understanding of ROS 2 nodes and resource management

### Objective
Create a node that properly manages resources and handles cleanup.

### Steps
1. Create a node called `lifecycle_node`
2. In the node, open a file for writing
3. Set up a timer that writes data to the file periodically
4. Override the `destroy_node()` method to properly close the file
5. Test that the file is properly closed when the node is shut down
6. Add error handling for file operations

### Expected Outcome
- A node that properly manages file resources
- File is closed when node is destroyed
- Error handling for file operations
- Proper cleanup on shutdown

### Hints
- Use `hasattr()` to check if resources exist before trying to close them
- Always call `super().destroy_node()` at the end of your override
- Use try/except blocks around file operations