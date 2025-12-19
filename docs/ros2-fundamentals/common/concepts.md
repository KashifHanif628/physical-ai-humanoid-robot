# Common ROS 2 Concepts

This document provides a reference for common ROS 2 concepts used throughout the module.

## Core Concepts

### Node
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

### Topic
Topics are named buses over which nodes exchange messages. Publishers send messages to topics, and subscribers receive messages from topics.

### Service
Services provide a request/response communication pattern. A service client sends a request to a service server, which processes the request and returns a response.

### Action
Actions are a goal-oriented communication pattern with feedback. They are used for long-running tasks where progress feedback is needed.

### Package
A package is the basic building unit of ROS 2. It contains libraries, executables, and other resources needed for a specific functionality.

## Common Message Types

- `std_msgs`: Standard message types like String, Int32, Float64
- `geometry_msgs`: Messages for geometry like Pose, Twist, Point
- `sensor_msgs`: Messages for sensors like LaserScan, Image, JointState