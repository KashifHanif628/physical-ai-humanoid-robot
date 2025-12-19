"""
Path Planning Script for Humanoid Navigation

This script demonstrates path planning techniques specifically designed for humanoid robots,
considering bipedal gait patterns and stability constraints.
"""

import numpy as np
import math
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped


class HumanoidPathPlanner(Node):
    """
    A path planner specifically designed for humanoid robots with stability considerations
    """

    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Initialize action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Humanoid-specific parameters
        self.step_length = 0.3  # Distance between consecutive steps
        self.step_width = 0.2   # Lateral distance between feet
        self.max_step_height = 0.1  # Maximum step height for obstacles
        self.stability_margin = 0.15  # Safety margin for stability

        # Store current robot state
        self.current_pose = None
        self.is_moving = False

        self.get_logger().info('Humanoid Path Planner initialized')

    def calculate_bipedal_path(self, start_pose, goal_pose):
        """
        Calculate a path suitable for bipedal navigation

        Args:
            start_pose: Starting pose of the robot
            goal_pose: Goal pose to navigate to

        Returns:
            List of poses representing the bipedal path
        """
        path = []

        # Calculate the straight-line path
        dx = goal_pose.position.x - start_pose.position.x
        dy = goal_pose.position.y - start_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate the number of steps needed
        num_steps = int(distance / self.step_length) + 1

        # Generate intermediate poses along the path
        for i in range(num_steps + 1):
            ratio = i / num_steps if num_steps > 0 else 0
            intermediate_pose = Pose()
            intermediate_pose.position.x = start_pose.position.x + dx * ratio
            intermediate_pose.position.y = start_pose.position.y + dy * ratio
            intermediate_pose.position.z = start_pose.position.z

            # For orientation, we'll use the goal orientation for the final pose
            # and interpolate for intermediate poses
            if i == num_steps:  # Final pose
                intermediate_pose.orientation = goal_pose.orientation
            else:  # Intermediate poses
                intermediate_pose.orientation = start_pose.orientation

            path.append(intermediate_pose)

        return path

    def add_stability_constraints(self, path):
        """
        Add stability constraints to the path for humanoid locomotion

        Args:
            path: List of poses to add stability constraints to

        Returns:
            Path with added stability constraints
        """
        constrained_path = []

        for i, pose in enumerate(path):
            # Add stability margin around each pose
            stable_pose = Pose()
            stable_pose.position = pose.position
            stable_pose.orientation = pose.orientation

            # Add additional poses if needed for stability transitions
            if i > 0:
                # Calculate direction vector from previous pose
                prev_pose = path[i-1]
                dx = pose.position.x - prev_pose.position.x
                dy = pose.position.y - prev_pose.position.y
                distance = math.sqrt(dx**2 + dy**2)

                # If the step is too large, add intermediate poses
                if distance > self.step_length * 1.5:
                    # Add intermediate pose at half distance
                    mid_pose = Pose()
                    mid_pose.position.x = prev_pose.position.x + dx * 0.5
                    mid_pose.position.y = prev_pose.position.y + dy * 0.5
                    mid_pose.position.z = prev_pose.position.z
                    mid_pose.orientation = prev_pose.orientation
                    constrained_path.append(mid_pose)

            constrained_path.append(stable_pose)

        return constrained_path

    def generate_footstep_plan(self, path):
        """
        Generate a footstep plan based on the navigation path

        Args:
            path: List of poses for navigation

        Returns:
            List of left and right foot positions
        """
        left_foot_steps = []
        right_foot_steps = []

        for i, pose in enumerate(path):
            # Alternate between left and right foot
            if i % 2 == 0:  # Even indices: left foot
                left_pos = Point()
                left_pos.x = pose.position.x
                left_pos.y = pose.position.y + self.step_width / 2
                left_pos.z = pose.position.z
                left_foot_steps.append(left_pos)
            else:  # Odd indices: right foot
                right_pos = Point()
                right_pos.x = pose.position.x
                right_pos.y = pose.position.y - self.step_width / 2
                right_pos.z = pose.position.z
                right_foot_steps.append(right_pos)

        return left_foot_steps, right_foot_steps

    def plan_path_to_pose(self, goal_x, goal_y, goal_yaw=0.0):
        """
        Plan a path to a specific pose

        Args:
            goal_x: X coordinate of the goal
            goal_y: Y coordinate of the goal
            goal_yaw: Yaw angle of the goal (optional)

        Returns:
            List of poses representing the planned path
        """
        # Create goal pose
        goal_pose = Pose()
        goal_pose.position.x = goal_x
        goal_pose.position.y = goal_y
        goal_pose.position.z = 0.0

        # Convert yaw to quaternion (simplified)
        goal_pose.orientation.z = math.sin(goal_yaw / 2)
        goal_pose.orientation.w = math.cos(goal_yaw / 2)

        # For this example, we'll use a dummy start pose
        # In a real implementation, you'd get the current robot pose
        start_pose = Pose()
        start_pose.position.x = 0.0
        start_pose.position.y = 0.0
        start_pose.position.z = 0.0
        start_pose.orientation.z = 0.0
        start_pose.orientation.w = 1.0

        # Calculate the bipedal path
        raw_path = self.calculate_bipedal_path(start_pose, goal_pose)

        # Add stability constraints
        stable_path = self.add_stability_constraints(raw_path)

        return stable_path

    def send_navigation_goal(self, goal_pose):
        """
        Send a navigation goal to the Nav2 system

        Args:
            goal_pose: The goal pose to navigate to
        """
        # Wait for the action server to be available
        self.nav_to_pose_client.wait_for_server()

        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = goal_pose

        # Send the goal
        self.nav_to_pose_client.send_goal_async(goal_msg)


def main(args=None):
    """
    Main function to demonstrate the humanoid path planner
    """
    rclpy.init(args=args)

    path_planner = HumanoidPathPlanner()

    # Example: Plan a path to a specific location
    goal_x, goal_y = 5.0, 3.0
    path = path_planner.plan_path_to_pose(goal_x, goal_y)

    path_planner.get_logger().info(f'Planned path with {len(path)} waypoints')

    # Generate footstep plan
    left_steps, right_steps = path_planner.generate_footstep_plan(path)
    path_planner.get_logger().info(f'Generated {len(left_steps)} left steps and {len(right_steps)} right steps')

    # For this example, we'll just print the path
    # In a real implementation, you would send the path to the navigation system
    for i, pose in enumerate(path):
        print(f"Waypoint {i}: ({pose.position.x:.2f}, {pose.position.y:.2f})")

    # Example of sending a navigation goal (commented out to avoid actual navigation)
    # if path:
    #     path_planner.send_navigation_goal(path[-1])

    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()