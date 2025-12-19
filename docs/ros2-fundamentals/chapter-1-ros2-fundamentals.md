---
title: ROS 2 Introduction and Installation
sidebar_label: Chapter 1: ROS 2 Fundamentals
description: Learn the basics of ROS 2 and how to set up your development environment
keywords: [ros2, installation, setup, fundamentals, robotics]
---

# ROS 2 Introduction and Installation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand what ROS 2 is and why it's important for robotics
- Install ROS 2 Humble Hawksbill on your system
- Set up a basic ROS 2 workspace
- Run your first ROS 2 commands
- Troubleshoot common installation issues

## Prerequisites

Before starting this chapter, you should:
- Have a Linux system (Ubuntu 22.04 recommended) or a virtual machine
- Have at least 4GB of RAM and 10GB of free disk space
- Have basic command line knowledge
- Have basic Python knowledge (Python 3.8+)

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system, but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 provides:
- A way to distribute computation across processes and devices
- A way to package your software
- A way to simulate your robot
- A way to visualize and debug your robot's behavior
- A way to interface with hardware
- A large community of robot software developers

### Key Concepts in ROS 2

**Nodes**: A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system.

**Topics**: Topics are named buses over which nodes exchange messages. Publishers send messages to topics, and subscribers receive messages from topics.

**Services**: Services provide a request/response communication pattern. A service client sends a request to a service server, which processes the request and returns a response.

**Actions**: Actions are a goal-oriented communication pattern with feedback. They are used for long-running tasks where progress feedback is needed.

## Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the current long-term support (LTS) distribution with 5 years of support until May 2027. It has the most stable and well-documented features, making it ideal for learners.

### Step 1: Set up locale

```bash
locale  # check for UTF-8
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Setup sources

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Step 4: Environment setup

```bash
source /opt/ros/humble/setup.bash
```

To automatically source ROS 2 environment in new terminals:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 5: Install Python Development Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

## Creating a ROS 2 Workspace

A workspace is a directory where you modify, build, and install ROS 2 packages.

### Step 1: Create the workspace

```bash
mkdir -p ~/ros2_fundamentals_ws/src
cd ~/ros2_fundamentals_ws
colcon build
source install/setup.bash
```

To automatically source your workspace:

```bash
echo "source ~/ros2_fundamentals_ws/install/setup.bash" >> ~/.bashrc
```

## Running Basic ROS 2 Commands

### Check ROS 2 installation

```bash
ros2 --version
```

### List available commands

```bash
ros2
```

### Run a simple demo

Terminal 1 - run a talker node:
```bash
source ~/ros2_fundamentals_ws/install/setup.bash
ros2 run demo_nodes_cpp talker
```

Terminal 2 - run a listener node:
```bash
source ~/ros2_fundamentals_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

## Troubleshooting

For common issues and solutions, please refer to our troubleshooting guide in the ROS 2 documentation.

## Summary

In this chapter, you've learned:
- What ROS 2 is and why it's important
- How to install ROS 2 Humble Hawksbill
- How to set up a basic ROS 2 workspace
- How to run basic ROS 2 commands

Now that you have a working ROS 2 environment, you're ready to dive deeper into ROS 2 concepts in the next chapter.