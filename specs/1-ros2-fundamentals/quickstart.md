# Quickstart Guide: ROS 2 Fundamentals Module

**Feature**: 1-ros2-fundamentals
**Version**: 1.0
**Created**: 2025-12-17

## Overview

This quickstart guide provides a complete path for learners to get started with the ROS 2 fundamentals module. By following this guide, you'll have a working ROS 2 environment and run your first examples.

## Prerequisites

Before starting, ensure you have:
- A Linux system (Ubuntu 22.04 recommended) or a virtual machine
- At least 4GB of RAM
- At least 10GB of free disk space
- Internet connection for package downloads
- Basic command line knowledge
- Basic Python knowledge (Python 3.8+)

## Step 1: Install ROS 2 Humble Hawksbill

### 1.1 Set up locale
```bash
locale  # check for UTF-8
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2 Setup sources
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 1.4 Environment setup
```bash
source /opt/ros/humble/setup.bash
```

To automatically source ROS 2 environment in new terminals:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Step 2: Install Python Development Tools

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

## Step 3: Create a ROS 2 Workspace

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

## Step 4: Run Your First ROS 2 Commands

### 4.1 Check ROS 2 installation
```bash
ros2 --version
```

### 4.2 List available commands
```bash
ros2
```

### 4.3 Run a simple demo
```bash
# Terminal 1 - run a talker node
source ~/ros2_fundamentals_ws/install/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2 - run a listener node
source ~/ros2_fundamentals_ws/install/setup.bash
ros2 run demo_nodes_cpp listener
```

## Step 5: Create Your First Python Node

### 5.1 Create a package
```bash
cd ~/ros2_fundamentals_ws/src
ros2 pkg create --build-type ament_python my_first_package
```

### 5.2 Create a simple publisher
Create the file `~/ros2_fundamentals_ws/src/my_first_package/my_first_package/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.3 Update setup.py
In `~/ros2_fundamentals_ws/src/my_first_package/setup.py`, add the following entry points:

```python
entry_points={
    'console_scripts': [
        'simple_publisher = my_first_package.simple_publisher:main',
    ],
},
```

### 5.4 Build and run
```bash
cd ~/ros2_fundamentals_ws
colcon build --packages-select my_first_package
source install/setup.bash
ros2 run my_first_package simple_publisher
```

### 5.5 Run a subscriber in another terminal
```bash
# Terminal 2
source ~/ros2_fundamentals_ws/install/setup.bash
ros2 run demo_nodes_py listener
```

## Step 6: Verify Your Setup

Run these commands to verify everything is working:

```bash
# Check ROS 2 nodes
ros2 node list

# Check ROS 2 topics
ros2 topic list

# Check specific topic info
ros2 topic info /chatter
```

## Next Steps

Now that you have a working ROS 2 environment, continue with the full module:

1. **Chapter 1**: Complete the ROS 2 fundamentals chapter to understand core concepts
2. **Chapter 2**: Learn Python control with rclpy for more advanced node development
3. **Chapter 3**: Explore humanoid URDF fundamentals to understand robot modeling

## Troubleshooting

### Common Issues

**Issue**: "command not found" for ROS 2 commands
**Solution**: Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

**Issue**: Python packages not found
**Solution**: Make sure you've installed the Python development tools and sourced your workspace

**Issue**: Permission errors during installation
**Solution**: Use `sudo` for system package installations, but not for workspace operations

**Issue**: Nodes can't communicate between terminals
**Solution**: Ensure ROS 2 environment is sourced in each terminal

## Getting Help

- Check the official ROS 2 documentation: https://docs.ros.org/en/humble/
- Visit the ROS answers forum: https://answers.ros.org/
- Join the ROS discourse: https://discourse.ros.org/