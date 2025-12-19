"""
Voice Command Mapping for VLA Systems

This script demonstrates how to map voice commands to ROS 2 actions
for Vision-Language-Action robotic systems.
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import re


class VoiceCommandMapper:
    """
    A class to map voice commands to ROS 2 actions for robotic execution
    """

    def __init__(self):
        rospy.init_node('voice_command_mapper')

        # Publishers for different types of commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/voice_status', String, queue_size=10)

        # Action clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Subscribe to voice commands
        self.voice_sub = rospy.Subscriber('/voice_command', String, self.voice_command_callback)

        # Command mapping patterns
        self.command_patterns = {
            # Navigation commands
            'move_forward': {
                'patterns': [r'go forward', r'move forward', r'forward', r'go ahead', r'straight'],
                'action': self.execute_move_forward
            },
            'move_backward': {
                'patterns': [r'go backward', r'move backward', r'backward', r'back'],
                'action': self.execute_move_backward
            },
            'turn_left': {
                'patterns': [r'turn left', r'left', r'rotate left'],
                'action': self.execute_turn_left
            },
            'turn_right': {
                'patterns': [r'turn right', r'right', r'rotate right'],
                'action': self.execute_turn_right
            },
            'stop': {
                'patterns': [r'stop', r'halt', r'pause', r'freeze'],
                'action': self.execute_stop
            },
            'go_to_location': {
                'patterns': [r'go to (.+)', r'move to (.+)', r'navigate to (.+)'],
                'action': self.execute_go_to_location
            }
        }

        rospy.loginfo('Voice command mapper initialized')

    def voice_command_callback(self, msg):
        """
        Callback for processing incoming voice commands

        Args:
            msg: String message containing the transcribed voice command
        """
        command = msg.data.lower().strip()
        rospy.loginfo(f'Received voice command: {command}')

        # Try to match command to patterns
        intent = self.match_command_pattern(command)

        if intent:
            # Publish status
            status_msg = String()
            status_msg.data = f"Processing: {intent}"
            self.status_pub.publish(status_msg)

            # Execute the matched action
            action = self.command_patterns[intent]['action']
            action(command)
        else:
            rospy.logwarn(f'Unknown command: {command}')
            self.publish_status(f"Unknown command: {command}")

    def match_command_pattern(self, command):
        """
        Match a command to known patterns

        Args:
            command: The voice command string

        Returns:
            The matched intent type or None if no match found
        """
        for intent, data in self.command_patterns.items():
            for pattern in data['patterns']:
                if re.search(pattern, command):
                    return intent
        return None

    def extract_location(self, command):
        """
        Extract location from command using regex

        Args:
            command: The voice command string

        Returns:
            Extracted location string or None
        """
        # Look for location patterns in the command
        patterns = [
            r'go to (.+)',
            r'move to (.+)',
            r'navigate to (.+)'
        ]

        for pattern in patterns:
            match = re.search(pattern, command)
            if match:
                location = match.group(1).strip()
                return location

        return None

    def execute_move_forward(self, command):
        """
        Execute move forward action

        Args:
            command: The original command string
        """
        twist = Twist()
        twist.linear.x = 0.5  # Forward speed
        self.cmd_vel_pub.publish(twist)
        self.publish_status("Moving forward")

    def execute_move_backward(self, command):
        """
        Execute move backward action

        Args:
            command: The original command string
        """
        twist = Twist()
        twist.linear.x = -0.5  # Backward speed
        self.cmd_vel_pub.publish(twist)
        self.publish_status("Moving backward")

    def execute_turn_left(self, command):
        """
        Execute turn left action

        Args:
            command: The original command string
        """
        twist = Twist()
        twist.angular.z = 0.5  # Left turn speed
        self.cmd_vel_pub.publish(twist)
        self.publish_status("Turning left")

    def execute_turn_right(self, command):
        """
        Execute turn right action

        Args:
            command: The original command string
        """
        twist = Twist()
        twist.angular.z = -0.5  # Right turn speed
        self.cmd_vel_pub.publish(twist)
        self.publish_status("Turning right")

    def execute_stop(self, command):
        """
        Execute stop action

        Args:
            command: The original command string
        """
        twist = Twist()
        # All velocities are 0 by default
        self.cmd_vel_pub.publish(twist)
        self.publish_status("Stopping")

    def execute_go_to_location(self, command):
        """
        Execute go to location action

        Args:
            command: The original command string
        """
        location = self.extract_location(command)
        if location:
            self.publish_status(f"Going to {location}")
            # In a real implementation, this would convert location to coordinates
            # and send a navigation goal to move_base
            self.send_navigation_goal(location)
        else:
            self.publish_status("Could not determine location")

    def send_navigation_goal(self, location):
        """
        Send navigation goal to move_base action server

        Args:
            location: The target location name
        """
        # This is a placeholder - in a real system, you would have
        # a map of location names to coordinates
        location_coordinates = {
            'kitchen': (1.0, 0.0, 0.0),  # x, y, theta
            'living room': (2.0, 1.0, 1.57),
            'bedroom': (-1.0, 2.0, 3.14),
        }

        if location in location_coordinates:
            x, y, theta = location_coordinates[location]

            # Wait for the action server to be available
            if self.move_base_client.wait_for_server(rospy.Duration(5.0)):
                # Create a goal
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()

                # Set the position
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                goal.target_pose.pose.orientation.z = theta

                # Send the goal
                self.move_base_client.send_goal(goal)
                self.publish_status(f"Navigation goal sent to {location}")
            else:
                self.publish_status("Navigation server not available")
        else:
            self.publish_status(f"Unknown location: {location}")

    def publish_status(self, status):
        """
        Publish status message

        Args:
            status: Status string to publish
        """
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        rospy.loginfo(status)


def main():
    """
    Main function to demonstrate voice command mapping
    """
    mapper = VoiceCommandMapper()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down voice command mapper")


if __name__ == '__main__':
    main()