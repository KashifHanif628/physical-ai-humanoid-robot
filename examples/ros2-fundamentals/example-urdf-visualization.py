#!/usr/bin/env python3

"""
URDF Visualization Example

This script demonstrates how to load and visualize a URDF model in ROS 2.
Note: This is a conceptual example. Actual URDF visualization typically requires
RViz2 or other visualization tools in the ROS 2 ecosystem.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math


class URDFVisualizationNode(Node):

    def __init__(self):
        super().__init__('urdf_visualization_node')

        # Publisher for joint states (needed for visualization in RViz2)
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Timer to publish joint states at 30Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_joint_states)

        # Initialize joint positions
        self.joint_positions = {
            'torso_to_head': 0.0,
            'torso_to_right_upper_arm': 0.0,
            'right_upper_arm_to_lower_arm': 0.0,
            'torso_to_left_upper_arm': 0.0,
            'left_upper_arm_to_lower_arm': 0.0,
            'torso_to_right_upper_leg': 0.0,
            'right_upper_leg_to_lower_leg': 0.0,
            'torso_to_left_upper_leg': 0.0,
            'left_upper_leg_to_lower_leg': 0.0
        }

        self.time = 0.0
        self.get_logger().info('URDF Visualization Node Started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Animate some joints with simple oscillating motion
        self.time += 0.01
        self.joint_positions['torso_to_right_upper_arm'] = math.sin(self.time) * 0.2
        self.joint_positions['torso_to_left_upper_arm'] = math.sin(self.time + math.pi) * 0.2
        self.joint_positions['torso_to_head'] = math.sin(self.time * 0.5) * 0.1

        # Publish the message
        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = URDFVisualizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down URDF Visualization Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()