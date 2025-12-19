"""
Safety Monitoring Node for VLA Systems

This script implements comprehensive safety monitoring
for Vision-Language-Action robotic systems.
"""

import rospy
import threading
import time
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, JointState
from nav_msgs.msg import Odometry
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np


class SafetyMonitor:
    """
    A ROS node that implements comprehensive safety monitoring for VLA systems
    """

    def __init__(self):
        rospy.init_node('safety_monitor')

        # Safety parameters
        self.velocity_limits = {
            'linear': 1.0,   # m/s
            'angular': 0.5   # rad/s
        }

        self.acceleration_limits = {
            'linear': 2.0,   # m/s^2
            'angular': 1.0   # rad/s^2
        }

        self.joint_limits = {
            'min': -3.14,    # radians
            'max': 3.14      # radians
        }

        self.torque_limit = 100.0  # N·m

        # Robot state tracking
        self.current_twist = Twist()
        self.previous_twist = Twist()
        self.current_pose = Pose()
        self.current_joints = JointState()
        self.laser_scan = LaserScan()

        # Timestamps for acceleration calculation
        self.prev_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Publishers and subscribers
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        self.safety_status_pub = rospy.Publisher('/safety_status', String, queue_size=10)
        self.velocity_cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.velocity_cmd_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Action clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Safety state
        self.emergency_stop_active = False
        self.safety_violation = False
        self.violation_reason = ""

        # Monitoring thread
        self.monitoring_thread = threading.Thread(target=self.monitor_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

        rospy.loginfo('Safety Monitor initialized')

    def velocity_cmd_callback(self, msg):
        """
        Callback for velocity commands to check safety limits

        Args:
            msg: Twist message containing velocity commands
        """
        if self.emergency_stop_active:
            return

        # Check velocity limits
        if self.exceeds_velocity_limit(msg):
            self.trigger_safety_violation(f"Velocity limit exceeded: linear={msg.linear.x}, angular={msg.angular.z}")
            return

        self.current_twist = msg

    def odom_callback(self, msg):
        """
        Callback for odometry data

        Args:
            msg: Odometry message containing pose and twist data
        """
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def joint_state_callback(self, msg):
        """
        Callback for joint state data

        Args:
            msg: JointState message containing joint information
        """
        self.current_joints = msg

    def scan_callback(self, msg):
        """
        Callback for laser scan data

        Args:
            msg: LaserScan message containing obstacle detection data
        """
        self.laser_scan = msg

    def exceeds_velocity_limit(self, twist_cmd):
        """
        Check if velocity command exceeds limits

        Args:
            twist_cmd: Twist message with commanded velocities

        Returns:
            Boolean indicating if velocity limits are exceeded
        """
        linear_speed = abs(twist_cmd.linear.x)
        angular_speed = abs(twist_cmd.angular.z)

        if linear_speed > self.velocity_limits['linear']:
            return True
        if angular_speed > self.velocity_limits['angular']:
            return True

        return False

    def exceeds_acceleration_limit(self):
        """
        Check if acceleration exceeds limits

        Returns:
            Boolean indicating if acceleration limits are exceeded
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        if dt <= 0:
            return False

        # Calculate accelerations
        linear_acc = abs(self.current_twist.linear.x - self.previous_twist.linear.x) / dt
        angular_acc = abs(self.current_twist.angular.z - self.previous_twist.angular.z) / dt

        if linear_acc > self.acceleration_limits['linear']:
            return True
        if angular_acc > self.acceleration_limits['angular']:
            return True

        return False

    def joints_exceed_limits(self):
        """
        Check if joint positions exceed limits

        Returns:
            Boolean indicating if joint limits are exceeded
        """
        for position in self.current_joints.position:
            if position < self.joint_limits['min'] or position > self.joint_limits['max']:
                return True
        return False

    def detects_collision_risk(self):
        """
        Check if there's a collision risk based on laser scan

        Returns:
            Boolean indicating if collision risk is detected
        """
        if not self.laser_scan.ranges:
            return False

        # Check for obstacles within safe distance
        safe_distance = 0.5  # meters
        for range_val in self.laser_scan.ranges:
            if 0 < range_val < safe_distance:
                return True

        return False

    def check_torque_limits(self):
        """
        Check if torque limits are exceeded (simulated)

        Returns:
            Boolean indicating if torque limits are exceeded
        """
        # In a real implementation, this would check actual torque sensors
        # For simulation, we'll return False
        return False

    def trigger_safety_violation(self, reason):
        """
        Trigger safety violation and emergency stop

        Args:
            reason: String describing the reason for safety violation
        """
        rospy.logerr(f"Safety violation: {reason}")

        self.safety_violation = True
        self.violation_reason = reason

        # Activate emergency stop
        self.activate_emergency_stop()

        # Publish emergency stop command
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

        # Publish safety status
        status_msg = String()
        status_msg.data = f"EMERGENCY STOP: {reason}"
        self.safety_status_pub.publish(status_msg)

    def activate_emergency_stop(self):
        """
        Activate emergency stop procedures
        """
        self.emergency_stop_active = True

        # Cancel any ongoing navigation goals
        if self.move_base_client.get_state() in [1, 2, 3, 4, 5, 6, 7]:  # Active states
            self.move_base_client.cancel_all_goals()

        # Stop all motion
        stop_twist = Twist()
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=True)
        cmd_vel_pub.publish(stop_twist)

        rospy.loginfo("Emergency stop activated")

    def deactivate_emergency_stop(self):
        """
        Deactivate emergency stop (manual override)
        """
        self.emergency_stop_active = False
        self.safety_violation = False
        self.violation_reason = ""

        # Publish safety status
        status_msg = String()
        status_msg.data = "NORMAL OPERATION"
        self.safety_status_pub.publish(status_msg)

        rospy.loginfo("Emergency stop deactivated")

    def monitor_loop(self):
        """
        Continuous monitoring loop
        """
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.emergency_stop_active:
                # Check various safety conditions
                if self.exceeds_acceleration_limit():
                    self.trigger_safety_violation("Acceleration limit exceeded")
                elif self.joints_exceed_limits():
                    self.trigger_safety_violation("Joint position limits exceeded")
                elif self.check_torque_limits():
                    self.trigger_safety_violation("Torque limits exceeded")
                elif self.detects_collision_risk():
                    self.trigger_safety_violation("Collision risk detected")

            # Update timestamps for acceleration calculation
            self.previous_twist = self.current_twist
            self.prev_time = self.current_time
            self.current_time = rospy.Time.now()

            rate.sleep()

    def get_safety_status(self):
        """
        Get current safety status

        Returns:
            Tuple of (is_safe, reason) where is_safe is boolean and reason is string
        """
        if self.emergency_stop_active:
            return False, "Emergency stop active: " + self.violation_reason
        if self.safety_violation:
            return False, self.violation_reason
        return True, "Normal operation"


def main():
    """
    Main function to demonstrate safety monitoring
    """
    safety_monitor = SafetyMonitor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down safety monitor")


if __name__ == '__main__':
    main()