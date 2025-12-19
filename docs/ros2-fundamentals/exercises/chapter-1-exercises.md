---
title: Chapter 1 Exercises - ROS 2 Introduction and Installation
sidebar_label: Chapter 1 Exercises
description: Exercises to verify your ROS 2 installation and basic understanding
keywords: ros2, exercises, installation, verification
---

# Chapter 1 Exercises - ROS 2 Introduction and Installation

## Exercise 1: Installation Verification

**Difficulty**: Beginner
**Estimated Time**: 15 minutes
**Prerequisites**: Completed Chapter 1 installation steps

### Objective
Verify that your ROS 2 installation is working correctly.

### Steps
1. Open a new terminal window
2. Check your ROS 2 version with `ros2 --version`
3. List available ROS 2 commands with `ros2`
4. List available packages with `ros2 pkg list | grep demo`
5. Verify you can run a basic command like `ros2 topic list`

### Expected Outcome
- You should see the ROS 2 version number (should be Humble Hawksbill)
- You should see a list of available ROS 2 commands
- You should see various demo packages listed
- You should see an empty topic list (no active nodes running)

### Hints
- Make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
- If commands are not found, check your `.bashrc` file to ensure the ROS 2 setup is properly configured

---

## Exercise 2: Workspace Creation and Verification

**Difficulty**: Beginner
**Estimated Time**: 20 minutes
**Prerequisites**: Completed Chapter 1 workspace setup

### Objective
Create and verify a basic ROS 2 workspace.

### Steps
1. Create a new workspace directory: `mkdir -p ~/my_first_ros2_ws/src`
2. Navigate to the workspace: `cd ~/my_first_ros2_ws`
3. Build the workspace: `colcon build`
4. Source the workspace: `source install/setup.bash`
5. Check that the workspace environment is active by checking the `AMENT_PREFIX_PATH` variable

### Expected Outcome
- Workspace directory structure is created successfully
- Build completes without errors
- Environment variables are set correctly (check with `printenv | grep AMENT`)

### Hints
- Make sure you're in the workspace root directory when running `colcon build`
- The first build may take a few minutes as it sets up the build environment

---

## Exercise 3: Running Your First Nodes

**Difficulty**: Beginner
**Estimated Time**: 25 minutes
**Prerequisites**: Completed Chapter 1 installation and workspace setup

### Objective
Run a publisher and subscriber node to verify ROS 2 communication.

### Steps
1. Open two terminal windows
2. In both terminals, navigate to your workspace and source the setup: `source ~/ros2_fundamentals_ws/install/setup.bash`
3. In Terminal 1, run: `ros2 run demo_nodes_cpp talker`
4. In Terminal 2, run: `ros2 run demo_nodes_cpp listener`
5. Observe the communication between the nodes
6. Stop both nodes with Ctrl+C
7. Verify the nodes were running with `ros2 node list`
8. Verify the topic was active with `ros2 topic list`

### Expected Outcome
- Talker node publishes messages at 1Hz
- Listener node receives and displays these messages
- When you run `ros2 node list`, you see both nodes when they're running
- When you run `ros2 topic list`, you see `/chatter` topic when nodes are running

### Hints
- Make sure to source the workspace in both terminals
- The talker and listener communicate via the `/chatter` topic
- If nodes don't communicate, check that you sourced the environment in both terminals