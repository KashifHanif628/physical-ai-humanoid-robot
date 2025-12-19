---
title: "Humanoid Robot Kinematics & Control Systems"
sidebar_label: "Week 6: Humanoid Kinematics & Control"
description: "Learn about humanoid robot kinematics, bipedal locomotion, and control systems"
keywords: ["humanoid", "kinematics", "locomotion", "control", "bipedal", "balance", "manipulation"]
---

# Week 6: Humanoid Robot Kinematics & Control Systems

## Introduction

This week focuses on humanoid robot kinematics and control systems, which are critical components for Physical AI applications involving humanoid robots. You'll learn about forward and inverse kinematics, bipedal locomotion, balance control, and manipulation systems. These concepts enable humanoid robots to move naturally and perform complex tasks in human environments.

## Humanoid Robot Kinematics

Humanoid robots have complex kinematic structures with multiple degrees of freedom (DOF) that enable human-like movement. Understanding kinematics is crucial for controlling these robots effectively.

### Forward Kinematics

Forward kinematics calculates the end-effector position from joint angles. For humanoid robots, this involves calculating the position of hands, feet, and head based on joint configurations.

```python
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class HumanoidForwardKinematics:
    def __init__(self):
        # Define humanoid robot dimensions (approximate human proportions)
        self.link_lengths = {
            'torso': 0.5,      # Torso height
            'upper_arm': 0.3,  # Upper arm length
            'forearm': 0.25,   # Forearm length
            'thigh': 0.4,      # Thigh length
            'shin': 0.38,      # Shin length
            'foot': 0.25       # Foot length
        }

    def calculate_arm_fk(self, shoulder_pos, joint_angles, arm='left'):
        """
        Calculate forward kinematics for arm (shoulder -> elbow -> wrist)

        Args:
            shoulder_pos: Position of shoulder joint [x, y, z]
            joint_angles: [shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch]
            arm: 'left' or 'right' (affects sign of some angles)

        Returns:
            Dictionary with positions of shoulder, elbow, wrist
        """
        # Convert angles to radians
        sy, sp, sr, ep = [math.radians(angle) for angle in joint_angles]

        # Shoulder rotation matrix
        shoulder_rot = self.euler_to_rotation_matrix([sy, sp, sr])

        # Calculate elbow position relative to shoulder
        elbow_offset = np.array([0, 0, -self.link_lengths['upper_arm']])
        elbow_rel = shoulder_rot @ elbow_offset
        elbow_pos = shoulder_pos + elbow_rel

        # Elbow rotation (only pitch for simplicity)
        elbow_rot = self.euler_to_rotation_matrix([0, ep, 0])

        # Calculate wrist position relative to elbow
        wrist_offset = np.array([0, 0, -self.link_lengths['forearm']])
        wrist_rel = elbow_rot @ wrist_offset
        wrist_pos = elbow_pos + wrist_rel

        return {
            'shoulder': shoulder_pos,
            'elbow': elbow_pos,
            'wrist': wrist_pos
        }

    def calculate_leg_fk(self, hip_pos, joint_angles, leg='left'):
        """
        Calculate forward kinematics for leg (hip -> knee -> ankle)

        Args:
            hip_pos: Position of hip joint [x, y, z]
            joint_angles: [hip_yaw, hip_pitch, hip_roll, knee_pitch]
            leg: 'left' or 'right' (affects sign of some angles)

        Returns:
            Dictionary with positions of hip, knee, ankle
        """
        # Convert angles to radians
        hy, hp, hr, kp = [math.radians(angle) for angle in joint_angles]

        # Hip rotation matrix
        hip_rot = self.euler_to_rotation_matrix([hy, hp, hr])

        # Calculate knee position relative to hip
        knee_offset = np.array([0, 0, -self.link_lengths['thigh']])
        knee_rel = hip_rot @ knee_offset
        knee_pos = hip_pos + knee_rel

        # Knee rotation (only pitch for simplicity)
        knee_rot = self.euler_to_rotation_matrix([0, kp, 0])

        # Calculate ankle position relative to knee
        ankle_offset = np.array([0, 0, -self.link_lengths['shin']])
        ankle_rel = knee_rot @ ankle_offset
        ankle_pos = knee_pos + ankle_rel

        return {
            'hip': hip_pos,
            'knee': knee_pos,
            'ankle': ankle_pos
        }

    def euler_to_rotation_matrix(self, angles):
        """
        Convert Euler angles (yaw, pitch, roll) to rotation matrix
        """
        yaw, pitch, roll = angles

        # Rotation matrices for each axis
        Ryaw = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])

        Rpitch = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        Rroll = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        # Combined rotation matrix
        return Ryaw @ Rpitch @ Rroll
```

### Inverse Kinematics

Inverse kinematics calculates the joint angles needed to reach a desired end-effector position. This is essential for humanoid robots to perform reaching and manipulation tasks.

```python
import numpy as np
from scipy.optimize import minimize

class HumanoidInverseKinematics:
    def __init__(self):
        # Robot dimensions
        self.upper_arm_len = 0.3
        self.forearm_len = 0.25
        self.thigh_len = 0.4
        self.shin_len = 0.38

    def calculate_arm_ik(self, target_pos, shoulder_pos, initial_guess=None):
        """
        Calculate inverse kinematics for arm using numerical optimization

        Args:
            target_pos: Target position for end effector [x, y, z]
            shoulder_pos: Shoulder position [x, y, z]
            initial_guess: Initial joint angle guess [shoulder_yaw, shoulder_pitch, shoulder_roll, elbow_pitch]

        Returns:
            Joint angles that achieve the target position (or closest approximation)
        """
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0]  # [yaw, pitch, roll, elbow_pitch]

        def objective_function(joint_angles):
            # Calculate current end-effector position
            current_pos = self.calculate_arm_fk_numerical(shoulder_pos, joint_angles)

            # Calculate distance to target
            distance = np.linalg.norm(np.array(current_pos) - np.array(target_pos))

            # Add penalty for joint angle limits
            angle_penalty = sum(max(0, abs(angle) - 180) for angle in joint_angles)

            return distance + 0.1 * angle_penalty  # Weighted combination

        # Optimize to find joint angles
        result = minimize(objective_function, initial_guess, method='BFGS')

        return result.x

    def calculate_arm_fk_numerical(self, shoulder_pos, joint_angles):
        """
        Numerical forward kinematics for arm (used in IK optimization)
        """
        # Convert to radians
        sy, sp, sr, ep = [math.radians(angle) for angle in joint_angles]

        # Calculate elbow position
        elbow_x = shoulder_pos[0] + self.upper_arm_len * math.sin(sp) * math.cos(sy)
        elbow_y = shoulder_pos[1] + self.upper_arm_len * math.sin(sp) * math.sin(sy)
        elbow_z = shoulder_pos[2] - self.upper_arm_len * math.cos(sp)

        # Calculate wrist position (simplified)
        wrist_x = elbow_x + self.forearm_len * math.sin(sp + ep) * math.cos(sy)
        wrist_y = elbow_y + self.forearm_len * math.sin(sp + ep) * math.sin(sy)
        wrist_z = elbow_z - self.forearm_len * math.cos(sp + ep)

        return [wrist_x, wrist_y, wrist_z]

    def calculate_leg_ik(self, target_pos, hip_pos, initial_guess=None):
        """
        Calculate inverse kinematics for leg using numerical optimization

        Args:
            target_pos: Target position for foot [x, y, z]
            hip_pos: Hip position [x, y, z]
            initial_guess: Initial joint angle guess [hip_yaw, hip_pitch, hip_roll, knee_pitch]

        Returns:
            Joint angles that achieve the target position (or closest approximation)
        """
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0]  # [yaw, pitch, roll, knee_pitch]

        def objective_function(joint_angles):
            # Calculate current foot position
            current_pos = self.calculate_leg_fk_numerical(hip_pos, joint_angles)

            # Calculate distance to target
            distance = np.linalg.norm(np.array(current_pos) - np.array(target_pos))

            # Add penalty for joint angle limits
            angle_penalty = sum(max(0, abs(angle) - 90) for angle in joint_angles)

            return distance + 0.1 * angle_penalty

        # Optimize to find joint angles
        result = minimize(objective_function, initial_guess, method='BFGS')

        return result.x

    def calculate_leg_fk_numerical(self, hip_pos, joint_angles):
        """
        Numerical forward kinematics for leg (used in IK optimization)
        """
        # Convert to radians
        hy, hp, hr, kp = [math.radians(angle) for angle in joint_angles]

        # Calculate knee position
        knee_x = hip_pos[0] + self.thigh_len * math.sin(hp) * math.cos(hy)
        knee_y = hip_pos[1] + self.thigh_len * math.sin(hp) * math.sin(hy)
        knee_z = hip_pos[2] - self.thigh_len * math.cos(hp)

        # Calculate ankle position
        ankle_x = knee_x + self.shin_len * math.sin(hp + kp) * math.cos(hy)
        ankle_y = knee_y + self.shin_len * math.sin(hp + kp) * math.sin(hy)
        ankle_z = knee_z - self.shin_len * math.cos(hp + kp)

        return [ankle_x, ankle_y, ankle_z]
```

## Bipedal Locomotion

Bipedal locomotion is one of the most challenging aspects of humanoid robotics. Unlike wheeled robots, humanoid robots must maintain balance while walking with only two contact points.

### Center of Mass and Stability

```python
import numpy as np

class BipedalStability:
    def __init__(self):
        # Robot parameters
        self.total_mass = 50.0  # kg
        self.com_height = 0.8  # m (center of mass height)
        self.foot_width = 0.1  # m
        self.foot_length = 0.25  # m

        # Support polygon (area where COM must stay for stability)
        self.support_polygon = self.calculate_support_polygon()

    def calculate_support_polygon(self):
        """
        Calculate the support polygon based on foot positions
        """
        # For double support phase (both feet on ground)
        # Approximate as rectangle between feet
        return {
            'min_x': -self.foot_length / 2,
            'max_x': self.foot_length / 2,
            'min_y': -self.foot_width,  # Both feet width
            'max_y': self.foot_width
        }

    def is_stable(self, com_position, foot_positions):
        """
        Check if robot is stable based on COM position

        Args:
            com_position: Center of mass position [x, y, z]
            foot_positions: List of foot positions [[x1, y1, z1], [x2, y2, z2]]

        Returns:
            Boolean indicating stability
        """
        # Calculate support polygon based on current foot positions
        support_poly = self.calculate_current_support_polygon(foot_positions)

        # Check if COM projection is within support polygon
        com_xy = [com_position[0], com_position[1]]

        return (support_poly['min_x'] <= com_xy[0] <= support_poly['max_x'] and
                support_poly['min_y'] <= com_xy[1] <= support_poly['max_y'])

    def calculate_current_support_polygon(self, foot_positions):
        """
        Calculate support polygon based on current foot positions
        """
        if len(foot_positions) == 0:
            return {'min_x': 0, 'max_x': 0, 'min_y': 0, 'max_y': 0}
        elif len(foot_positions) == 1:
            # Single support - smaller polygon around single foot
            foot = foot_positions[0]
            return {
                'min_x': foot[0] - self.foot_length/2,
                'max_x': foot[0] + self.foot_length/2,
                'min_y': foot[1] - self.foot_width/2,
                'max_y': foot[1] + self.foot_width/2
            }
        else:
            # Double support - polygon encompassing both feet
            x_coords = [foot[0] for foot in foot_positions]
            y_coords = [foot[1] for foot in foot_positions]

            return {
                'min_x': min(x_coords) - self.foot_length/2,
                'max_x': max(x_coords) + self.foot_length/2,
                'min_y': min(y_coords) - self.foot_width/2,
                'max_y': max(y_coords) + self.foot_width/2
            }

    def calculate_zmp(self, forces_moments):
        """
        Calculate Zero Moment Point (ZMP) for stability analysis

        Args:
            forces_moments: Dictionary with force and moment values
                           {'fx', 'fy', 'fz', 'mx', 'my', 'mz'}

        Returns:
            ZMP position [x, y]
        """
        fz = forces_moments['fz']
        mx = forces_moments['mx']
        my = forces_moments['my']

        if abs(fz) < 1e-6:  # Avoid division by zero
            return [0, 0]

        zmp_x = -my / fz
        zmp_y = mx / fz

        return [zmp_x, zmp_y]
```

### Walking Pattern Generation

```python
import numpy as np
import math

class WalkingPatternGenerator:
    def __init__(self):
        # Walking parameters
        self.step_length = 0.3      # m
        self.step_width = 0.2      # m (distance between feet)
        self.step_height = 0.05    # m (foot lift height)
        self.walk_period = 1.0     # s (time for one step)
        self.dsp_ratio = 0.2       # Double Support Phase ratio
        self.com_height = 0.8      # m (desired COM height)

        # Trajectory parameters
        self.trajectory_points = 50

    def generate_walk_trajectory(self, num_steps, direction='forward'):
        """
        Generate walking trajectory for bipedal locomotion

        Args:
            num_steps: Number of steps to generate
            direction: 'forward', 'backward', 'left', 'right', 'turn_left', 'turn_right'

        Returns:
            Dictionary with trajectories for both feet and COM
        """
        trajectories = {
            'left_foot': [],
            'right_foot': [],
            'com': [],
            'support_foot': []  # Which foot is in support phase
        }

        # Initialize starting positions
        left_foot_start = np.array([0, self.step_width/2, 0])
        right_foot_start = np.array([0, -self.step_width/2, 0])
        com_start = np.array([0, 0, self.com_height])

        for step in range(num_steps):
            # Generate single step trajectory
            step_trajectories = self.generate_single_step(
                left_foot_start, right_foot_start, com_start, direction, step
            )

            # Append to overall trajectories
            trajectories['left_foot'].extend(step_trajectories['left_foot'])
            trajectories['right_foot'].extend(step_trajectories['right_foot'])
            trajectories['com'].extend(step_trajectories['com'])
            trajectories['support_foot'].extend(step_trajectories['support_foot'])

            # Update starting positions for next step
            left_foot_start = step_trajectories['left_foot'][-1]
            right_foot_start = step_trajectories['right_foot'][-1]
            com_start = step_trajectories['com'][-1]

        return trajectories

    def generate_single_step(self, left_start, right_start, com_start, direction, step_num):
        """
        Generate trajectory for a single step
        """
        # Calculate step displacement based on direction
        if direction == 'forward':
            step_displacement = np.array([self.step_length, 0, 0])
        elif direction == 'backward':
            step_displacement = np.array([-self.step_length, 0, 0])
        elif direction == 'left':
            step_displacement = np.array([0, self.step_width, 0])
        elif direction == 'right':
            step_displacement = np.array([0, -self.step_width, 0])
        elif direction == 'turn_left':
            step_displacement = np.array([self.step_length * 0.7, self.step_width * 0.5, 0])
        elif direction == 'turn_right':
            step_displacement = np.array([self.step_length * 0.7, -self.step_width * 0.5, 0])
        else:
            step_displacement = np.array([0, 0, 0])

        # Determine which foot is swing foot (moving) vs support foot
        if step_num % 2 == 0:  # Even steps: left foot swings
            swing_foot_start = left_start
            stance_foot_pos = right_start
            swing_foot_final = left_start + step_displacement
        else:  # Odd steps: right foot swings
            swing_foot_start = right_start
            stance_foot_pos = left_start
            swing_foot_final = right_start + step_displacement

        # Generate swing foot trajectory (circular arc)
        swing_trajectory = self.generate_swing_trajectory(
            swing_foot_start, swing_foot_final
        )

        # Generate stance foot trajectory (stays in place during swing phase)
        stance_trajectory = [stance_foot_pos] * len(swing_trajectory)

        # Generate COM trajectory (follows inverted pendulum model)
        com_trajectory = self.generate_com_trajectory(
            com_start, step_displacement, len(swing_trajectory)
        )

        # Determine support foot for each time step
        if step_num % 2 == 0:
            support_foot = ['right'] * len(swing_trajectory)  # Left foot swinging, right supporting
        else:
            support_foot = ['left'] * len(swing_trajectory)   # Right foot swinging, left supporting

        # Organize trajectories based on step number
        if step_num % 2 == 0:  # Even steps: left foot moves
            left_trajectory = swing_trajectory
            right_trajectory = stance_trajectory
        else:  # Odd steps: right foot moves
            left_trajectory = stance_trajectory
            right_trajectory = swing_trajectory

        return {
            'left_foot': left_trajectory,
            'right_foot': right_trajectory,
            'com': com_trajectory,
            'support_foot': support_foot
        }

    def generate_swing_trajectory(self, start_pos, end_pos):
        """
        Generate swing foot trajectory (parabolic arc)
        """
        trajectory = []

        for i in range(self.trajectory_points):
            t = i / (self.trajectory_points - 1)  # Normalized time [0, 1]

            # Parabolic trajectory with foot lift
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])

            # Vertical trajectory (parabolic lift)
            z_lift = self.step_height * math.sin(math.pi * t)  # Sinusoidal lift
            z = start_pos[2] + t * (end_pos[2] - start_pos[2]) + z_lift

            trajectory.append(np.array([x, y, z]))

        return trajectory

    def generate_com_trajectory(self, start_pos, step_displacement, num_points):
        """
        Generate COM trajectory following inverted pendulum model
        """
        trajectory = []

        # Calculate intermediate positions
        for i in range(num_points):
            t = i / (num_points - 1)  # Normalized time [0, 1]

            # Horizontal movement follows step pattern
            x = start_pos[0] + t * (step_displacement[0])
            y = start_pos[1] + t * (step_displacement[1])

            # Keep COM height relatively constant
            z = self.com_height

            trajectory.append(np.array([x, y, z]))

        return trajectory
```

## Balance Control Systems

Maintaining balance is critical for humanoid robots. Various control strategies are employed to keep the robot stable during movement and in response to disturbances.

### PID Balance Controller

```python
import numpy as np

class BalanceController:
    def __init__(self):
        # PID gains for balance control
        self.kp = 100.0   # Proportional gain
        self.ki = 10.0    # Integral gain
        self.kd = 50.0    # Derivative gain

        # Balance thresholds
        self.com_threshold = 0.1  # Maximum COM deviation (m)
        self.angle_threshold = 0.3  # Maximum tilt angle (rad)

        # Internal state
        self.error_integral = 0.0
        self.previous_error = 0.0
        self.control_output = 0.0

    def update_balance(self, current_com_error, current_angular_error, dt):
        """
        Update balance control based on COM and angular errors

        Args:
            current_com_error: Current COM position error [x, y]
            current_angular_error: Current angular error [roll, pitch, yaw]
            dt: Time step

        Returns:
            Control output for joint adjustments
        """
        # Use pitch error for balance control (primary balance axis)
        error = current_angular_error[1]  # Pitch error

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.error_integral += error * dt
        i_term = self.ki * self.error_integral

        # Derivative term
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        d_term = self.kd * derivative

        # PID output
        output = p_term + i_term + d_term

        # Apply saturation limits
        max_output = 50.0  # Maximum control effort
        output = max(min(output, max_output), -max_output)

        # Update state
        self.previous_error = error
        self.control_output = output

        return output

    def calculate_balance_adjustment(self, robot_state):
        """
        Calculate balance adjustment based on robot state

        Args:
            robot_state: Dictionary with robot state information
                        {'com_position', 'orientation', 'angular_velocity',
                         'foot_positions', 'forces_torques'}

        Returns:
            Dictionary with balance adjustments for joints
        """
        # Calculate COM error relative to support polygon
        com_error = self.calculate_com_error(robot_state)

        # Calculate angular error from upright position
        angular_error = self.calculate_angular_error(robot_state['orientation'])

        # Calculate balance adjustments
        adjustments = {
            'left_hip_roll': 0.0,
            'right_hip_roll': 0.0,
            'left_ankle_roll': 0.0,
            'right_ankle_roll': 0.0,
            'left_ankle_pitch': 0.0,
            'right_ankle_pitch': 0.0,
            'torso_pitch': 0.0
        }

        # Adjust based on angular error (primary balance correction)
        adjustments['left_ankle_roll'] = -angular_error[0] * 0.5  # Roll correction
        adjustments['right_ankle_roll'] = -angular_error[0] * 0.5
        adjustments['left_ankle_pitch'] = -angular_error[1] * 0.3  # Pitch correction
        adjustments['right_ankle_pitch'] = -angular_error[1] * 0.3
        adjustments['torso_pitch'] = -angular_error[1] * 0.2

        # Adjust based on COM position (secondary correction)
        adjustments['left_hip_roll'] = -com_error[1] * 0.8  # Lateral COM correction
        adjustments['right_hip_roll'] = -com_error[1] * 0.8
        adjustments['torso_pitch'] += -com_error[0] * 0.4  # Forward/backward correction

        return adjustments

    def calculate_com_error(self, robot_state):
        """
        Calculate COM position error relative to support polygon
        """
        com_pos = np.array(robot_state['com_position'])
        foot_positions = robot_state['foot_positions']

        # Calculate support polygon center
        if len(foot_positions) > 0:
            support_center_x = sum(foot[0] for foot in foot_positions) / len(foot_positions)
            support_center_y = sum(foot[1] for foot in foot_positions) / len(foot_positions)
        else:
            support_center_x, support_center_y = 0, 0

        # Calculate error
        error_x = com_pos[0] - support_center_x
        error_y = com_pos[1] - support_center_y

        return [error_x, error_y]

    def calculate_angular_error(self, orientation):
        """
        Calculate angular error from upright position
        """
        # Assuming orientation is in quaternion format [w, x, y, z]
        qw, qx, qy, qz = orientation

        # Convert to roll, pitch, yaw
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Error from upright (0, 0, 0)
        return [roll, pitch, yaw]
```

## Manipulation and Grasping

Humanoid robots need effective manipulation capabilities to interact with objects in human environments.

### Grasp Planning

```python
import numpy as np
from enum import Enum

class GraspType(Enum):
    PALM_GRASP = "palm_grasp"
    PINCER_GRASP = "pincer_grasp"
    THREE_FINGER_GRASP = "three_finger_grasp"
    SPHERICAL_GRASP = "spherical_grasp"

class GraspPlanner:
    def __init__(self):
        # Hand parameters
        self.finger_length = 0.1
        self.thumb_length = 0.08
        self.hand_width = 0.12
        self.max_force = 50.0  # N

        # Grasp database (simplified)
        self.grasp_database = {
            'cylinder': [GraspType.PALM_GRASP, GraspType.PINCER_GRASP],
            'box': [GraspType.PALM_GRASP, GraspType.THREE_FINGER_GRASP],
            'sphere': [GraspType.SPHERICAL_GRASP, GraspType.THREE_FINGER_GRASP],
            'handle': [GraspType.PALM_GRASP]
        }

    def plan_grasp(self, object_info):
        """
        Plan grasp for an object based on its properties

        Args:
            object_info: Dictionary with object properties
                        {'shape', 'size', 'weight', 'center_of_mass', 'friction_coefficient'}

        Returns:
            Grasp plan with joint angles and forces
        """
        # Determine best grasp type based on object properties
        best_grasp = self.select_best_grasp(object_info)

        # Generate grasp configuration
        grasp_config = self.generate_grasp_configuration(best_grasp, object_info)

        return {
            'grasp_type': best_grasp,
            'configuration': grasp_config,
            'approach_vector': self.calculate_approach_vector(object_info),
            'grasp_quality': self.evaluate_grasp_quality(grasp_config, object_info)
        }

    def select_best_grasp(self, object_info):
        """
        Select best grasp type based on object properties
        """
        shape = object_info['shape']
        weight = object_info['weight']
        size = object_info['size']  # [width, height, depth]

        # Determine possible grasps for this object
        possible_grasps = self.grasp_database.get(shape, [GraspType.PINCER_GRASP])

        # Select best grasp based on object properties
        if weight > 2.0:  # Heavy object - use palm grasp for stability
            if GraspType.PALM_GRASP in possible_grasps:
                return GraspType.PALM_GRASP

        if size[0] < 0.05 and size[1] < 0.05:  # Small object - use pincer grasp
            if GraspType.PINCER_GRASP in possible_grasps:
                return GraspType.PINCER_GRASP

        # Default to first available grasp
        return possible_grasps[0]

    def generate_grasp_configuration(self, grasp_type, object_info):
        """
        Generate joint configuration for specified grasp type
        """
        if grasp_type == GraspType.PALM_GRASP:
            return self.generate_palm_grasp_config(object_info)
        elif grasp_type == GraspType.PINCER_GRASP:
            return self.generate_pincer_grasp_config(object_info)
        elif grasp_type == GraspType.THREE_FINGER_GRASP:
            return self.generate_three_finger_grasp_config(object_info)
        elif grasp_type == GraspType.SPHERICAL_GRASP:
            return self.generate_spherical_grasp_config(object_info)
        else:
            return self.generate_default_grasp_config()

    def generate_palm_grasp_config(self, object_info):
        """
        Generate configuration for palm grasp
        """
        # Palm grasp - wrap fingers around object
        return {
            'thumb_angle': 45.0,  # degrees
            'index_flexion': 90.0,
            'middle_flexion': 90.0,
            'ring_flexion': 90.0,
            'pinky_flexion': 60.0,
            'fingers_spread': 10.0  # degrees between fingers
        }

    def generate_pincer_grasp_config(self, object_info):
        """
        Generate configuration for pincer grasp (thumb + index finger)
        """
        return {
            'thumb_angle': 30.0,
            'index_flexion': 120.0,
            'middle_flexion': 0.0,  # Not used in pincer
            'ring_flexion': 0.0,
            'pinky_flexion': 0.0,
            'fingers_spread': 0.0
        }

    def calculate_approach_vector(self, object_info):
        """
        Calculate approach vector for safe grasping
        """
        # For now, return a safe approach from above
        # In practice, this would consider object shape and environment
        return [0.0, 0.0, 1.0]  # Approach from above

    def evaluate_grasp_quality(self, grasp_config, object_info):
        """
        Evaluate the quality of a grasp configuration
        """
        # Calculate grasp quality based on various factors
        weight = object_info['weight']
        friction = object_info['friction_coefficient']

        # Base quality score
        quality = 0.5

        # Adjust based on object weight
        if weight < 1.0:
            quality += 0.2  # Light objects are easier to grasp
        elif weight > 5.0:
            quality -= 0.2  # Heavy objects are harder to grasp securely

        # Adjust based on friction
        quality += friction * 0.3  # Higher friction improves grasp

        # Clamp to [0, 1]
        quality = max(0.0, min(1.0, quality))

        return quality
```

## ROS 2 Integration for Humanoid Control

Here's how to integrate the humanoid kinematics and control systems with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller_node')

        # Initialize kinematics and control systems
        self.forward_kinematics = HumanoidForwardKinematics()
        self.inverse_kinematics = HumanoidInverseKinematics()
        self.balance_controller = BalanceController()
        self.walk_generator = WalkingPatternGenerator()
        self.grasp_planner = GraspPlanner()

        # Publishers
        self.joint_command_pub = self.create_publisher(
            JointState, '/joint_commands', 10
        )
        self.com_state_pub = self.create_publisher(
            Pose, '/com_state', 10
        )
        self.balance_state_pub = self.create_publisher(
            Float64MultiArray, '/balance_state', 10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.foot_pressure_sub = self.create_subscription(
            Float64MultiArray, '/foot_pressure', self.foot_pressure_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Timers for control loops
        self.balance_timer = self.create_timer(0.01, self.balance_control_loop)  # 100 Hz
        self.walk_timer = self.create_timer(0.02, self.walk_control_loop)  # 50 Hz
        self.manipulation_timer = self.create_timer(0.05, self.manipulation_control_loop)  # 20 Hz

        # Robot state
        self.current_joint_states = JointState()
        self.foot_pressure_data = []
        self.imu_data = Imu()

        self.get_logger().info('Humanoid Controller Node initialized')

    def joint_state_callback(self, msg):
        """
        Update current joint states
        """
        self.current_joint_states = msg
        self.update_robot_kinematics()

    def foot_pressure_callback(self, msg):
        """
        Update foot pressure sensor data
        """
        self.foot_pressure_data = msg.data

    def imu_callback(self, msg):
        """
        Update IMU data for balance control
        """
        self.imu_data = msg

    def update_robot_kinematics(self):
        """
        Update robot kinematic state based on joint positions
        """
        # Calculate current COM position based on joint angles
        com_position = self.calculate_com_from_joints(self.current_joint_states)

        # Publish COM state
        com_msg = Pose()
        com_msg.position.x = com_position[0]
        com_msg.position.y = com_position[1]
        com_msg.position.z = com_position[2]

        self.com_state_publisher.publish(com_msg)

    def balance_control_loop(self):
        """
        Balance control loop running at 100 Hz
        """
        # Get current robot state
        robot_state = self.get_current_robot_state()

        # Calculate balance adjustments
        balance_adjustments = self.balance_controller.calculate_balance_adjustment(robot_state)

        # Apply balance corrections
        self.apply_balance_corrections(balance_adjustments)

        # Publish balance state for monitoring
        balance_state_msg = Float64MultiArray()
        balance_state_msg.data = [
            robot_state['com_position'][0],
            robot_state['com_position'][1],
            robot_state['orientation'][1],  # Pitch angle
            self.balance_controller.control_output
        ]

        self.balance_state_publisher.publish(balance_state_msg)

    def walk_control_loop(self):
        """
        Walking control loop running at 50 Hz
        """
        # This would implement walking pattern generation and execution
        # For now, we'll just log that the loop is running
        pass

    def manipulation_control_loop(self):
        """
        Manipulation control loop running at 20 Hz
        """
        # This would implement manipulation and grasping control
        # For now, we'll just log that the loop is running
        pass

    def get_current_robot_state(self):
        """
        Get current robot state for control calculations
        """
        # Extract joint positions
        joint_positions = {}
        for i, name in enumerate(self.current_joint_states.name):
            joint_positions[name] = self.current_joint_states.position[i]

        # Calculate COM position (simplified)
        com_position = self.estimate_com_position(joint_positions)

        # Get orientation from IMU
        orientation = [
            self.imu_data.orientation.w,
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z
        ]

        return {
            'joint_positions': joint_positions,
            'com_position': com_position,
            'orientation': orientation,
            'angular_velocity': [
                self.imu_data.angular_velocity.x,
                self.imu_data.angular_velocity.y,
                self.imu_data.angular_velocity.z
            ],
            'foot_positions': self.calculate_foot_positions(joint_positions),
            'forces_torques': {}  # Would come from force/torque sensors
        }

    def estimate_com_position(self, joint_positions):
        """
        Estimate COM position based on joint configuration (simplified)
        """
        # This is a simplified estimation - in reality, this would use
        # a more complex dynamic model with link masses and positions
        return [0.0, 0.0, 0.8]  # Approximate COM height for standing humanoid

    def calculate_foot_positions(self, joint_positions):
        """
        Calculate foot positions based on leg joint angles
        """
        # This would use forward kinematics to calculate foot positions
        # For now, return placeholder positions
        return [
            [0.1, 0.1, 0.0],  # Left foot
            [0.1, -0.1, 0.0]  # Right foot
        ]

    def apply_balance_corrections(self, adjustments):
        """
        Apply balance corrections to joint commands
        """
        # Create joint command message
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = list(adjustments.keys())
        joint_cmd.position = list(adjustments.values())

        # Apply to current joint positions with adjustments
        adjusted_positions = []
        for i, joint_name in enumerate(self.current_joint_states.name):
            if joint_name in adjustments:
                base_pos = self.current_joint_states.position[
                    self.current_joint_states.name.index(joint_name)
                ]
                adj_pos = base_pos + np.radians(adjustments[joint_name])
                adjusted_positions.append(adj_pos)
            else:
                adjusted_positions.append(self.current_joint_states.position[i])

        joint_cmd.position = adjusted_positions
        self.joint_command_publisher.publish(joint_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidControllerNode()

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

## Launch File for Humanoid Control

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Humanoid controller node
    humanoid_controller = Node(
        package='physical_ai_humanoid_control',
        executable='humanoid_controller_node',
        name='humanoid_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/joint_states', '/physical_ai_humanoid/joint_states'),
            ('/joint_commands', '/physical_ai_humanoid/joint_commands'),
            ('/imu/data', '/physical_ai_humanoid/imu'),
            ('/foot_pressure', '/physical_ai_humanoid/foot_pressure'),
        ],
        output='screen'
    )

    # Kinematics solver node
    kinematics_solver = Node(
        package='physical_ai_humanoid_control',
        executable='kinematics_solver_node',
        name='kinematics_solver',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Balance controller node
    balance_controller = Node(
        package='physical_ai_humanoid_control',
        executable='balance_controller_node',
        name='balance_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'kp': 100.0},
            {'ki': 10.0},
            {'kd': 50.0},
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)

    # Add nodes
    ld.add_action(humanoid_controller)
    ld.add_action(kinematics_solver)
    ld.add_action(balance_controller)

    return ld
```

## Best Practices for Humanoid Control

### 1. Safety First
- Implement multiple layers of safety checks
- Use joint position and velocity limits
- Monitor for hardware failures and respond appropriately
- Include emergency stop mechanisms

### 2. Robust Control
- Use sensor fusion for more reliable state estimation
- Implement disturbance rejection
- Design controllers that handle model uncertainties
- Test extensively in simulation before physical deployment

### 3. Computational Efficiency
- Optimize kinematic calculations for real-time performance
- Use appropriate numerical methods for inverse kinematics
- Consider model predictive control for complex motions
- Implement efficient trajectory generation algorithms

### 4. Adaptive Behavior
- Adjust control parameters based on task requirements
- Learn from experience to improve performance
- Adapt to different terrains and environments
- Handle unexpected situations gracefully

## Learning Objectives

By the end of Week 6, you should be able to:
1. Understand and implement forward and inverse kinematics for humanoid robots
2. Design and implement bipedal locomotion patterns and control systems
3. Create balance control algorithms to maintain humanoid stability
4. Plan and execute manipulation and grasping tasks
5. Integrate humanoid control systems with ROS 2 architecture

## Exercises

### Exercise 1: Kinematics Implementation (Beginner)
- **Time**: 60 minutes
- **Objective**: Implement basic forward and inverse kinematics for a simplified humanoid arm
- **Steps**: Create FK and IK solvers for a 3-DOF arm and test with various targets
- **Expected Outcome**: Working kinematics solvers that can reach specified positions

### Exercise 2: Balance Control (Intermediate)
- **Time**: 90 minutes
- **Objective**: Implement a balance controller that maintains humanoid stability
- **Steps**: Create PID-based balance controller and test with simulated disturbances
- **Expected Outcome**: Controller that keeps simulated humanoid stable during perturbations

### Exercise 3: Walking Pattern Generation (Advanced)
- **Time**: 120 minutes
- **Objective**: Implement a complete walking pattern generator for bipedal locomotion
- **Steps**: Create trajectory generator with support for different gaits and turns
- **Expected Outcome**: Humanoid that can walk forward, backward, and turn smoothly

## Summary

Week 6 covered humanoid robot kinematics and control systems, including forward and inverse kinematics, bipedal locomotion, balance control, and manipulation. You learned to implement mathematical models for robot movement, design control systems for stability, and integrate these systems with ROS 2. In Week 7, we'll explore manipulation and grasping in more detail, building on the foundation established in this week.