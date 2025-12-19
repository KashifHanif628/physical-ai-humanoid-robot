---
title: "High-Fidelity Interaction with Unity"
sidebar_label: "Chapter 3: Unity Integration"
description: "Learn to integrate Unity with ROS 2 for high-fidelity human-robot interaction and visualization"
keywords: [unity, ros2, integration, visualization, interaction, robotics]
---

# High-Fidelity Interaction with Unity

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand why Unity is valuable for human-robot interaction
- Set up the ROS-Unity bridge for communication
- Separate rendering from physics in simulation systems
- Visualize robot state in real-time using Unity
- Implement bidirectional communication between Unity and ROS 2

## Prerequisites

Before starting this chapter, you should:
- Have completed Chapters 1 and 2 (Physics Simulation and Sensor Simulation)
- Have a working ROS 2 environment
- Understand basic ROS 2 concepts (topics, services, actions)
- Have basic understanding of Unity (optional but helpful)

## Why Unity for Human-Robot Interaction?

Unity is a powerful 3D development platform that excels at creating high-fidelity visualizations and intuitive user interfaces. For robotics applications, Unity offers several advantages:

### High-Quality Graphics
- Realistic lighting, shadows, and materials
- Advanced rendering pipelines (URP, HDRP)
- Smooth animations and visual effects
- VR/AR compatibility for immersive experiences

### Interactive Interfaces
- Intuitive GUI systems
- Rich input handling (mouse, keyboard, gamepad, touch)
- Animation and transition systems
- User experience design tools

### Real-time Performance
- Efficient rendering engine
- Multi-platform deployment options
- Asset optimization tools
- Profiling and debugging capabilities

### Integration Capabilities
- Extensive plugin ecosystem
- Flexible networking options
- Scripting with C#
- Custom editor tools

However, Unity is not optimized for physics simulation, which is where Gazebo excels. This is why many robotics applications use a hybrid approach: Gazebo for accurate physics simulation and Unity for high-fidelity visualization.

## ROS-Unity Bridge Concepts

The ROS-Unity bridge enables communication between ROS 2 and Unity. The most common approach is using TCP/IP communication with a serialization format like JSON or Protobuf.

### Architecture Overview

```
[ROS 2 Nodes] <---> [TCP/IP Network] <---> [Unity Application]
      |                    |                       |
  ROS 2 Topics        ROS-TCP-Connector    Unity Scripts
  Services/Actions    (Unity Plugin)       (C# Scripts)
```

### Communication Patterns

The ROS-Unity bridge typically supports:

1. **Topic-based Communication**: Publish/subscribe pattern for continuous data streams
2. **Service-based Communication**: Request/response pattern for discrete operations
3. **Action-based Communication**: Goal-oriented communication with feedback

## Setting Up the ROS-Unity Bridge

### Installation Requirements

For the Unity side, you'll typically use the Unity Robotics Simulation Package:

1. **Unity Hub**: Install Unity Hub for managing Unity versions
2. **Unity Editor**: Install Unity 2022.3 LTS or later
3. **Unity Robotics Simulation Package**: Available via Unity Package Manager
4. **ROS-TCP-Connector**: Unity plugin for ROS communication
5. **ROS-TCP-Endpoint**: ROS 2 package for TCP endpoint

### ROS 2 Setup

Install the ROS-TCP-Endpoint package:

```bash
# Clone the repository
cd ~/ros2_fundamentals_ws/src
git clone -b ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build the workspace
cd ~/ros2_fundamentals_ws
colcon build
source install/setup.bash
```

### Unity Setup

1. Open Unity Hub and create a new 3D project
2. In the Package Manager, install:
   - Unity Robotics Simulation Package
   - Unity Transport
   - Any additional packages as needed

## Unity Scene Setup for Robot Visualization

### Basic Scene Structure

A typical Unity scene for robot visualization includes:

```
Scene Root
├── Main Camera
├── Directional Light
├── Robot Visualization (Prefab)
│   ├── Base Link
│   ├── Joint 1
│   ├── Joint 2
│   └── ...
├── Environment
└── ROS Connection Manager
```

### Creating a Robot Visualization Prefab

Here's how to structure a robot visualization in Unity:

1. **Create the Robot GameObject** as the root
2. **Add child objects** for each link
3. **Position and rotate** each link according to the robot's kinematics
4. **Add visual components** (MeshRenderer, Material, etc.)

### Example Robot Visualization Script

Create a C# script in Unity to visualize robot state:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotVisualizer : MonoBehaviour
{
    [SerializeField] private string robotName = "my_robot";
    [SerializeField] private Transform[] jointTransforms; // Array of joint transforms to control
    [SerializeField] private float[] jointPositions;      // Current joint positions
    [SerializeField] private float smoothingFactor = 10f; // For smooth interpolation

    private ROSConnection ros;
    private string jointStatesTopic = "/joint_states";

    void Start()
    {
        // Get the ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to joint states topic
        ros.Subscribe<sensor_msgs.JointStateMsg>(jointStatesTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(sensor_msgs.JointStateMsg jointState)
    {
        if (jointState.name.Length != jointState.position.Length)
        {
            Debug.LogError("Joint names and positions arrays have different lengths!");
            return;
        }

        // Update joint positions based on received state
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            float newPosition = jointState.position[i];

            // Find the corresponding transform and update its rotation
            for (int j = 0; j < jointTransforms.Length; j++)
            {
                if (jointTransforms[j].name == jointName)
                {
                    jointPositions[j] = newPosition;
                    break;
                }
            }
        }
    }

    void Update()
    {
        // Smoothly interpolate joint positions for visualization
        for (int i = 0; i < jointTransforms.Length && i < jointPositions.Length; i++)
        {
            // Example: Rotate around Z-axis for a revolute joint
            float targetRotation = jointPositions[i] * Mathf.Rad2Deg; // Convert radians to degrees
            float currentRotation = jointTransforms[i].localEulerAngles.z;
            float smoothedRotation = Mathf.Lerp(currentRotation, targetRotation,
                                              Time.deltaTime * smoothingFactor);

            Vector3 newEuler = jointTransforms[i].localEulerAngles;
            newEuler.z = smoothedRotation;
            jointTransforms[i].localEulerAngles = newEuler;
        }
    }
}
```

## Separation of Rendering vs Physics

One of the key benefits of the hybrid approach is separating rendering from physics:

### Physics Simulation (Gazebo)
- Accurate collision detection
- Realistic dynamics and constraints
- Stable physics integration
- Computational efficiency for physics

### Rendering (Unity)
- High-quality visuals
- Advanced lighting and materials
- Smooth animations
- User interface components

### Data Flow Between Systems

The data flow typically works as follows:

1. **Gazebo** simulates physics and robot dynamics
2. **ROS 2** publishes robot state (joint positions, sensor data)
3. **Unity** subscribes to ROS topics and updates visualization
4. **Unity** may send commands back to ROS 2 for robot control

## Example: Visualizing Robot State in Unity

Let's create a complete example of visualizing robot state:

### 1. Unity Script for Robot State Visualization

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class RobotStateVisualizer : MonoBehaviour
{
    [Header("Robot Configuration")]
    [SerializeField] private string robotNamespace = "/my_robot";

    [Header("Joint Visualization")]
    [SerializeField] private JointVisualization[] joints;

    [Header("Sensor Visualization")]
    [SerializeField] private SensorVisualization[] sensors;

    [Header("Performance")]
    [SerializeField] private float updateRate = 30f; // Hz
    [SerializeField] private float smoothingFactor = 15f;

    private ROSConnection ros;
    private float nextUpdateTime;

    [System.Serializable]
    public class JointVisualization
    {
        public string jointName;
        public Transform jointTransform;
        public JointType jointType = JointType.Revolute;
        public Vector3 rotationAxis = Vector3.forward;
        [Range(-180, 180)] public float minAngle = -90f;
        [Range(-180, 180)] public float maxAngle = 90f;
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }

    [System.Serializable]
    public class SensorVisualization
    {
        public string sensorName;
        public GameObject visualizationObject;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to joint states
        ros.Subscribe<Sensor.JointStateMsg>($"{robotNamespace}/joint_states", OnJointStatesReceived);

        // Subscribe to sensor data (example for IMU)
        ros.Subscribe<Sensor.ImuMsg>($"{robotNamespace}/imu/data", OnImuReceived);

        nextUpdateTime = Time.time;
    }

    void OnJointStatesReceived(Sensor.JointStateMsg jointState)
    {
        if (jointState.name.Count != jointState.position.Count)
        {
            Debug.LogError("Joint names and positions arrays have different lengths!");
            return;
        }

        // Update each joint's position
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            double newPosition = jointState.position[i];

            foreach (var joint in joints)
            {
                if (joint.jointName == jointName)
                {
                    UpdateJoint(joint, (float)newPosition);
                    break;
                }
            }
        }
    }

    void OnImuReceived(Sensor.ImuMsg imuData)
    {
        // Visualize IMU data (e.g., show orientation indicator)
        foreach (var sensor in sensors)
        {
            if (sensor.sensorName.Contains("imu"))
            {
                // Update IMU visualization based on orientation
                var orientation = new Quaternion(
                    (float)imuData.orientation.x,
                    (float)imuData.orientation.y,
                    (float)imuData.orientation.z,
                    (float)imuData.orientation.w
                );

                sensor.visualizationObject.transform.rotation = orientation;
                break;
            }
        }
    }

    void UpdateJoint(JointVisualization joint, float newPosition)
    {
        float targetValue = newPosition;

        switch (joint.jointType)
        {
            case JointType.Revolute:
                // Convert radians to degrees for Unity
                targetValue = Mathf.Rad2Deg * newPosition;
                break;
            case JointType.Prismatic:
                // Use position directly for prismatic joints
                break;
            case JointType.Fixed:
                // Fixed joints don't move
                return;
        }

        // Apply smoothing for visualization
        float currentValue = GetJointValue(joint);
        float smoothedValue = Mathf.Lerp(currentValue, targetValue,
                                       Time.deltaTime * smoothingFactor);

        SetJointValue(joint, smoothedValue);
    }

    float GetJointValue(JointVisualization joint)
    {
        switch (joint.jointType)
        {
            case JointType.Revolute:
                return joint.jointTransform.localEulerAngles.z;
            case JointType.Prismatic:
                return joint.jointTransform.localPosition.x;
            default:
                return 0f;
        }
    }

    void SetJointValue(JointVisualization joint, float value)
    {
        switch (joint.jointType)
        {
            case JointType.Revolute:
                Vector3 rotation = joint.jointTransform.localEulerAngles;
                rotation.z = value;
                joint.jointTransform.localEulerAngles = rotation;
                break;
            case JointType.Prismatic:
                Vector3 position = joint.jointTransform.localPosition;
                position.x = value;
                joint.jointTransform.localPosition = position;
                break;
        }
    }

    void Update()
    {
        // Limit update rate for performance
        if (Time.time < nextUpdateTime)
            return;

        nextUpdateTime = Time.time + 1f / updateRate;

        // Additional visualization updates can happen here
    }
}
```

### 2. Unity Script for Sending Commands to ROS 2

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    [Header("ROS Configuration")]
    [SerializeField] private string robotNamespace = "/my_robot";

    [Header("Control Configuration")]
    [SerializeField] private string cmdVelTopic = "cmd_vel";

    [Header("Input Configuration")]
    [SerializeField] private string forwardAxis = "Vertical";
    [SerializeField] private string turnAxis = "Horizontal";

    [Header("Movement Parameters")]
    [SerializeField] private float linearSpeed = 1.0f;
    [SerializeField] private float angularSpeed = 1.0f;

    private ROSConnection ros;
    private float forwardInput;
    private float turnInput;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    void Update()
    {
        // Get input from user
        forwardInput = Input.GetAxis(forwardAxis);
        turnInput = Input.GetAxis(turnAxis);

        // Send command if input is detected
        if (Mathf.Abs(forwardInput) > 0.1f || Mathf.Abs(turnInput) > 0.1f)
        {
            SendVelocityCommand();
        }
    }

    void SendVelocityCommand()
    {
        // Create Twist message for differential drive
        var twist = new TwistMsg();
        twist.linear.x = forwardInput * linearSpeed;
        twist.angular.z = turnInput * angularSpeed;

        // Send the command to ROS 2
        ros.Publish($"{robotNamespace}/{cmdVelTopic}", twist);
    }

    // Method to send joint position commands
    public void SendJointPositionCommand(string jointName, float position)
    {
        // Create joint trajectory message
        var jointTrajectory = new trajectory_msgs.JointTrajectoryMsg();
        jointTrajectory.joint_names.Add(jointName);

        var point = new trajectory_msgs.JointTrajectoryPointMsg();
        point.positions.Add(position);
        point.time_from_start = new builtin_interfaces.DurationMsg(1, 0); // 1 second

        jointTrajectory.points.Add(point);

        ros.Publish($"{robotNamespace}/joint_trajectory", jointTrajectory);
    }
}
```

## Bidirectional Communication Example

### Unity to ROS 2 (Commands)

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Actionlib;

public class UnityCommandSender : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
    }

    public void SendNavigationGoal(float x, float y, float theta)
    {
        // Example: Send navigation goal
        var goal = new geometry_msgs.PoseStampedMsg();
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0;

        // Convert theta to quaternion
        goal.pose.orientation = new geometry_msgs.QuaternionMsg(
            0, 0,
            Mathf.Sin(theta / 2),
            Mathf.Cos(theta / 2)
        );

        ros.Publish("/move_base_simple/goal", goal);
    }

    public void SendServiceRequest()
    {
        // Example: Send service request
        // This would require a custom service message type
    }
}
```

### ROS 2 to Unity (State Updates)

On the ROS 2 side, you'd have nodes that publish robot state:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Timer to publish state
        self.timer = self.create_timer(0.033, self.publish_state)  # ~30Hz

        # Robot state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        self.joint_positions = [0.0] * 6  # Example: 6 DOF robot arm

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vtheta = msg.angular.z

    def publish_state(self):
        # Update robot pose based on velocity
        dt = 0.033  # Timer period
        self.x += self.vx * dt * math.cos(self.theta) - self.vy * dt * math.sin(self.theta)
        self.y += self.vx * dt * math.sin(self.theta) + self.vy * dt * math.cos(self.theta)
        self.theta += self.vtheta * dt

        # Update joint positions (example: simple oscillation)
        t = self.get_clock().now().nanoseconds / 1e9
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * math.sin(t + i)

        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f'joint_{i}' for i in range(len(self.joint_positions))]
        joint_msg.position = self.joint_positions
        self.joint_pub.publish(joint_msg)

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.vtheta
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

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

## Best Practices for Unity-ROS Integration

### 1. Performance Optimization
- Limit update rates to reduce network traffic
- Use interpolation for smooth visualization
- Optimize Unity assets and rendering
- Implement efficient data serialization

### 2. Error Handling
- Implement connection status monitoring
- Add retry mechanisms for failed communications
- Log communication errors for debugging
- Provide fallback behaviors when connection is lost

### 3. Security Considerations
- Use secure network protocols when possible
- Validate data received from ROS 2
- Implement access controls for sensitive operations
- Monitor for unexpected data patterns

### 4. Scalability
- Design modular communication systems
- Use configuration files for topic/service names
- Implement connection pooling if needed
- Consider bandwidth limitations for remote operations

## Troubleshooting Unity-ROS Integration

### Common Issues and Solutions

**Issue**: Unity application cannot connect to ROS 2
**Solution**: Check that ROS-TCP-Endpoint is running: `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000`

**Issue**: Robot visualization is not updating
**Solution**: Verify topic names match between ROS 2 publisher and Unity subscriber

**Issue**: High latency in visualization
**Solution**: Reduce Unity update rate or optimize network connection

**Issue**: Joint positions are not synchronized
**Solution**: Check that joint names in Unity match those published by ROS 2

## Summary

In this chapter, you've learned:
- Why Unity is valuable for human-robot interaction and visualization
- How to set up the ROS-Unity bridge for bidirectional communication
- How to separate physics simulation (Gazebo) from rendering (Unity)
- How to visualize robot state in real-time using Unity
- How to implement bidirectional communication between Unity and ROS 2

This completes the Digital Twin module, providing you with a comprehensive understanding of physics simulation, sensor simulation, and high-fidelity visualization for robotics applications. You now have the foundation needed for Module 3 (Isaac) and advanced robotics applications.