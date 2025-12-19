---
title: "Manipulation & Grasping for Humanoid Robots"
sidebar_label: "Week 7: Manipulation & Grasping"
description: "Learn about robotic manipulation, grasping techniques, and dexterous control for humanoid robots"
keywords: ["manipulation", "grasping", "dexterous", "hand control", "robotics", "humanoid"]
---

# Week 7: Manipulation & Grasping for Humanoid Robots

## Introduction

This week focuses on manipulation and grasping systems for humanoid robots, essential capabilities for performing tasks in human environments. You'll learn about dexterous hand control, grasp planning, force control, and coordination between multiple degrees of freedom. These skills enable humanoid robots to interact with objects effectively, a critical component of Physical AI systems.

## Robotic Manipulation Fundamentals

Robotic manipulation involves the control of robot end-effectors to interact with objects in the environment. For humanoid robots, this encompasses both gross motor manipulation (moving heavy objects) and fine motor manipulation (precise positioning and delicate handling).

### Degrees of Freedom and Workspace

Humanoid robots typically have 7-8 DOF per arm to achieve human-like manipulation capabilities. This redundancy allows for:

- **Posture Optimization**: Achieving desired end-effector poses while optimizing joint configurations
- **Obstacle Avoidance**: Navigating around obstacles while maintaining task goals
- **Force Control**: Controlling contact forces during manipulation tasks
- **Dexterity**: Achieving complex manipulation patterns

### Manipulation Control Hierarchies

Humanoid manipulation typically employs multiple control levels:

1. **Task Space Control**: High-level control of end-effector position/orientation
2. **Joint Space Control**: Direct control of joint positions/velocities/torques
3. **Impedance Control**: Control of interaction forces and stiffness
4. **Grasp Control**: Fine control of fingertips and contact forces

## Dexterous Hand Control

Dexterous manipulation requires precise control of multi-fingered hands with multiple degrees of freedom.

### Hand Kinematics

Humanoid hands typically have 16-20 DOF distributed among fingers and thumb:

- **Thumb**: 4 DOF (opposition, flexion, abduction, rotation)
- **Index Finger**: 4 DOF (flexion, abduction, rotation, extension)
- **Middle Finger**: 4 DOF
- **Ring Finger**: 4 DOF
- **Pinky Finger**: 4 DOF

```python
import numpy as np
import math

class HumanoidHandController:
    def __init__(self, hand_type='left'):
        self.hand_type = hand_type
        self.finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']

        # Joint limits for each finger (in degrees)
        self.joint_limits = {
            'thumb': {'cmc_abduction': (-20, 20), 'cmc_flexion': (0, 60), 'mp_flexion': (0, 90), 'ip_flexion': (0, 90)},
            'index': {'mcp_flexion': (0, 90), 'pip_flexion': (0, 100), 'dip_flexion': (0, 100)},
            'middle': {'mcp_flexion': (0, 90), 'pip_flexion': (0, 100), 'dip_flexion': (0, 100)},
            'ring': {'mcp_flexion': (0, 90), 'pip_flexion': (0, 100), 'dip_flexion': (0, 100)},
            'pinky': {'mcp_flexion': (0, 90), 'pip_flexion': (0, 100), 'dip_flexion': (0, 100)}
        }

        # Hand dimensions (approximate)
        self.hand_dimensions = {
            'palm_width': 0.08,    # meters
            'palm_height': 0.04,   # meters
            'palm_thickness': 0.02, # meters
        }

    def calculate_finger_fk(self, finger_name, joint_angles):
        """
        Calculate forward kinematics for a single finger

        Args:
            finger_name: Name of the finger ('thumb', 'index', etc.)
            joint_angles: List of joint angles in degrees

        Returns:
            List of joint positions from palm to fingertip
        """
        if finger_name not in self.finger_names:
            raise ValueError(f"Unknown finger: {finger_name}")

        # Convert angles to radians
        angles_rad = [math.radians(angle) for angle in joint_angles]

        # Calculate joint positions (simplified model)
        joint_positions = []
        current_pos = np.array([0, 0, 0])  # Start at palm center

        # For this example, we'll use simplified link lengths
        link_lengths = {
            'thumb': [0.02, 0.03, 0.025],  # cmc, mp, ip
            'index': [0.03, 0.035, 0.03],  # mcp, pip, dip
            'middle': [0.03, 0.035, 0.03],
            'ring': [0.03, 0.035, 0.03],
            'pinky': [0.025, 0.03, 0.025]
        }

        for i, angle in enumerate(angles_rad):
            # Simplified: each joint moves in the same plane
            link_length = link_lengths[finger_name][i] if i < len(link_lengths[finger_name]) else 0.02
            delta_pos = np.array([
                link_length * math.cos(angle),
                link_length * math.sin(angle) * (1 if finger_name != 'pinky' else -1),  # Pinky curves differently
                0  # Simplified to 2D for this example
            ])
            current_pos += delta_pos
            joint_positions.append(current_pos.copy())

        return joint_positions

    def calculate_hand_pose(self, hand_config):
        """
        Calculate complete hand pose from configuration

        Args:
            hand_config: Dictionary with joint angles for all fingers

        Returns:
            Dictionary with positions of all joints and fingertips
        """
        hand_pose = {}

        for finger_name in self.finger_names:
            if finger_name in hand_config:
                joint_angles = hand_config[finger_name]
                finger_joints = self.calculate_finger_fk(finger_name, joint_angles)
                hand_pose[finger_name] = finger_joints

        return hand_pose

    def generate_grasp_configuration(self, grasp_type, object_properties):
        """
        Generate appropriate hand configuration for a specific grasp type

        Args:
            grasp_type: Type of grasp ('power', 'precision', 'pinch', etc.)
            object_properties: Dictionary with object properties

        Returns:
            Hand configuration dictionary
        """
        if grasp_type == 'power':
            # Power grasp - wrap fingers around object
            return {
                'thumb': [30, 60, 0, 0],  # [cmc_abduction, cmc_flexion, mp_flexion, ip_flexion]
                'index': [90, 100, 100],  # [mcp_flexion, pip_flexion, dip_flexion]
                'middle': [90, 100, 100],
                'ring': [90, 100, 100],
                'pinky': [90, 100, 100]
            }
        elif grasp_type == 'precision':
            # Precision grasp - tip-to-tip
            return {
                'thumb': [0, 45, 90, 90],
                'index': [0, 90, 90, 90],
                'middle': [0, 0, 0, 0],  # Not used in precision grasp
                'ring': [0, 0, 0, 0],
                'pinky': [0, 0, 0, 0]
            }
        elif grasp_type == 'cylindrical':
            # Cylindrical grasp for round objects
            return {
                'thumb': [15, 60, 75, 60],
                'index': [60, 90, 80, 70],
                'middle': [70, 90, 80, 70],
                'ring': [60, 85, 75, 65],
                'pinky': [45, 70, 60, 50]
            }
        else:
            # Default open hand
            return {
                'thumb': [0, 0, 0, 0],
                'index': [0, 0, 0, 0],
                'middle': [0, 0, 0, 0],
                'ring': [0, 0, 0, 0],
                'pinky': [0, 0, 0, 0]
            }

    def validate_grasp_configuration(self, config):
        """
        Validate that grasp configuration respects joint limits
        """
        for finger_name, angles in config.items():
            if finger_name in self.joint_limits:
                limits = self.joint_limits[finger_name]
                limit_keys = list(limits.keys())

                for i, angle in enumerate(angles):
                    if i < len(limit_keys):
                        min_limit, max_limit = limits[limit_keys[i]]
                        if not (min_limit <= angle <= max_limit):
                            return False, f"Joint {limit_keys[i]} of {finger_name} out of limits: {angle}° not in [{min_limit}, {max_limit}]"

        return True, "Configuration is valid"
```

## Grasp Planning and Execution

Grasp planning involves determining how to grasp an object based on its properties and the task requirements.

### Grasp Types and Selection

Different grasp types are appropriate for different objects and tasks:

```python
from enum import Enum
import numpy as np
from scipy.spatial import ConvexHull

class GraspType(Enum):
    POWER_GRASP = "power_grasp"
    PRECISION_GRASP = "precision_grasp"
    CYLINDRICAL_GRASP = "cylindrical_grasp"
    SPHERICAL_GRASP = "spherical_grasp"
    PINCER_GRASP = "pincer_grasp"
    HOOK_GRASP = "hook_grasp"

class GraspPlanner:
    def __init__(self):
        # Grasp quality evaluation weights
        self.quality_weights = {
            'force_closure': 0.4,
            'stability': 0.3,
            'dexterity': 0.2,
            'efficiency': 0.1
        }

    def plan_grasp(self, object_mesh, object_properties, task_requirements):
        """
        Plan optimal grasp for an object based on its properties and task requirements

        Args:
            object_mesh: Mesh representation of the object
            object_properties: Dictionary with object properties (weight, material, etc.)
            task_requirements: Dictionary with task-specific requirements

        Returns:
            Grasp plan with contact points, grasp type, and configuration
        """
        # Analyze object shape and properties
        shape_analysis = self.analyze_object_shape(object_mesh)

        # Determine suitable grasp types based on object properties
        candidate_grasps = self.generate_candidate_grasps(object_mesh, object_properties)

        # Evaluate grasp candidates
        best_grasp = self.evaluate_grasps(candidate_grasps, object_mesh, object_properties, task_requirements)

        # Generate hand configuration for best grasp
        hand_config = self.generate_hand_configuration(best_grasp)

        return {
            'grasp_type': best_grasp['type'],
            'contact_points': best_grasp['contact_points'],
            'approach_vector': best_grasp['approach'],
            'hand_configuration': hand_config,
            'quality_score': best_grasp['quality'],
            'safety_margin': 0.95  # Safety factor
        }

    def analyze_object_shape(self, mesh):
        """
        Analyze object shape to determine appropriate grasp strategies
        """
        # Calculate bounding box
        min_bounds = np.min(mesh.vertices, axis=0)
        max_bounds = np.max(mesh.vertices, axis=0)
        dimensions = max_bounds - min_bounds

        # Determine shape characteristics
        shape_features = {
            'dimensions': dimensions,
            'volume': self.calculate_volume(mesh),
            'surface_area': self.calculate_surface_area(mesh),
            'aspect_ratio': max(dimensions) / min(dimensions),
            'curvature': self.estimate_curvature(mesh),
            'symmetry': self.estimate_symmetry(mesh)
        }

        return shape_features

    def generate_candidate_grasps(self, mesh, object_properties):
        """
        Generate multiple candidate grasps for the object
        """
        candidates = []

        # Generate grasp candidates based on object shape
        if object_properties['dimensions'][0] > 2 * object_properties['dimensions'][1]:
            # Elongated object - prefer cylindrical grasp
            candidates.extend(self.generate_cylindrical_grasps(mesh))
        elif object_properties['dimensions'][0] < 0.05 and object_properties['dimensions'][1] < 0.05:
            # Small object - prefer precision grasp
            candidates.extend(self.generate_precision_grasps(mesh))
        else:
            # Medium-sized object - try multiple grasp types
            candidates.extend(self.generate_power_grasps(mesh))
            candidates.extend(self.generate_cylindrical_grasps(mesh))
            candidates.extend(self.generate_spherical_grasps(mesh))

        return candidates

    def generate_power_grasps(self, mesh):
        """
        Generate power grasp candidates
        """
        candidates = []

        # Find stable grasp points on the object
        contact_points = self.find_stable_contact_points(mesh)

        for points in contact_points:
            grasp = {
                'type': GraspType.POWER_GRASP,
                'contact_points': points,
                'approach': self.calculate_approach_vector(points),
                'quality': self.evaluate_power_grasp_quality(points, mesh)
            }
            candidates.append(grasp)

        return candidates

    def generate_precision_grasps(self, mesh):
        """
        Generate precision grasp candidates (thumb and finger tip)
        """
        candidates = []

        # Find suitable contact points for tip-to-tip grasping
        contact_pairs = self.find_precision_contact_pairs(mesh)

        for pair in contact_pairs:
            grasp = {
                'type': GraspType.PRECISION_GRASP,
                'contact_points': pair,
                'approach': self.calculate_approach_vector(pair),
                'quality': self.evaluate_precision_grasp_quality(pair, mesh)
            }
            candidates.append(grasp)

        return candidates

    def evaluate_grasp_quality(self, grasp, mesh, object_properties, task_requirements):
        """
        Evaluate quality of a grasp based on multiple factors
        """
        force_closure_score = self.evaluate_force_closure(grasp['contact_points'], mesh)
        stability_score = self.evaluate_stability(grasp, object_properties)
        dexterity_score = self.evaluate_dexterity(grasp, task_requirements)
        efficiency_score = self.evaluate_efficiency(grasp, object_properties)

        # Weighted combination
        quality = (
            self.quality_weights['force_closure'] * force_closure_score +
            self.quality_weights['stability'] * stability_score +
            self.quality_weights['dexterity'] * dexterity_score +
            self.quality_weights['efficiency'] * efficiency_score
        )

        return quality

    def evaluate_force_closure(self, contact_points, mesh):
        """
        Evaluate if the grasp achieves force closure (ability to resist external forces)
        """
        # For 3D objects, force closure requires at least 7 contact points
        # or special configurations with fewer points
        if len(contact_points) >= 7:
            return 1.0  # Good chance of force closure
        elif len(contact_points) >= 4:
            # Check if contacts span the object adequately
            hull = ConvexHull(np.array(contact_points))
            if len(hull.vertices) == len(contact_points):
                return 0.8  # Adequate coverage
        elif len(contact_points) >= 3:
            # Need special geometric configuration
            return 0.5

        return 0.2  # Poor force closure potential

    def evaluate_stability(self, grasp, object_properties):
        """
        Evaluate grasp stability considering object weight and center of mass
        """
        weight = object_properties.get('weight', 1.0)
        com = object_properties.get('center_of_mass', [0, 0, 0])

        # Calculate moment arm from contact points to COM
        contact_centroid = np.mean(grasp['contact_points'], axis=0)
        moment_arm = np.linalg.norm(np.array(com) - contact_centroid)

        # Stability decreases with moment arm and increases with weight
        stability = max(0, 1 - (moment_arm * weight) / 10.0)

        return stability

    def calculate_volume(self, mesh):
        """
        Calculate approximate volume of mesh
        """
        # Use convex hull approximation for speed
        hull = ConvexHull(mesh.vertices)
        return hull.volume

    def calculate_surface_area(self, mesh):
        """
        Calculate approximate surface area of mesh
        """
        hull = ConvexHull(mesh.vertices)
        return hull.area

    def estimate_curvature(self, mesh):
        """
        Estimate surface curvature at vertices
        """
        # Simplified: calculate average distance to neighbors
        avg_curvature = 0
        for vertex in mesh.vertices[:min(100, len(mesh.vertices))]:  # Sample points
            distances = [np.linalg.norm(vertex - other) for other in mesh.vertices[:10]]
            avg_curvature += sum(distances) / len(distances)

        return avg_curvature / min(100, len(mesh.vertices))
```

## Force Control and Compliance

Proper force control is essential for safe and effective manipulation, especially when interacting with humans or delicate objects.

### Impedance Control

Impedance control regulates the relationship between force and position, allowing robots to behave like springs with adjustable stiffness.

```python
import numpy as np

class ImpedanceController:
    def __init__(self):
        # Default impedance parameters
        self.mass = 1.0  # Equivalent mass
        self.damping = 2.0  # Damping coefficient
        self.stiffness = 1000.0  # Spring constant

        # Force limits
        self.max_force = 100.0  # N
        self.max_torque = 50.0  # Nm

        # Safety parameters
        self.force_deadband = 5.0  # N (acceptable force error)
        self.compliance_threshold = 0.01  # m (acceptable position error)

    def set_impedance_parameters(self, mass, damping, stiffness):
        """
        Set impedance parameters for different manipulation tasks
        """
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness

    def calculate_impedance_response(self, desired_pos, current_pos, desired_force, current_force, dt):
        """
        Calculate impedance control response

        Args:
            desired_pos: Desired position
            current_pos: Current position
            desired_force: Desired interaction force
            current_force: Current measured force
            dt: Time step

        Returns:
            Position adjustment to achieve desired impedance behavior
        """
        # Calculate position and force errors
        pos_error = desired_pos - current_pos
        force_error = desired_force - current_force

        # Impedance equation: M*a + B*v + K*x = F
        # Rearranging: a = (F - B*v - K*x) / M

        # Velocity (numerical derivative)
        velocity = (current_pos - self.previous_pos) / dt if dt > 0 else 0

        # Calculate acceleration
        acceleration = (force_error - self.damping * velocity - self.stiffness * pos_error) / self.mass

        # Integrate to get velocity and position changes
        delta_velocity = acceleration * dt
        delta_position = (self.current_velocity + delta_velocity) * dt

        # Update internal state
        self.previous_pos = current_pos
        self.current_velocity = self.current_velocity + delta_velocity

        return delta_position

    def adapt_impedance_for_task(self, task_type):
        """
        Adapt impedance parameters for different manipulation tasks
        """
        if task_type == 'delicate_handling':
            # Low stiffness for gentle interaction
            self.stiffness = 100.0
            self.damping = 1.0
            self.max_force = 10.0
        elif task_type == 'heavy_manipulation':
            # High stiffness for strong interaction
            self.stiffness = 2000.0
            self.damping = 5.0
            self.max_force = 200.0
        elif task_type == 'assembly':
            # Medium stiffness with precise control
            self.stiffness = 500.0
            self.damping = 3.0
            self.max_force = 50.0
        elif task_type == 'human_interaction':
            # Very compliant for safety
            self.stiffness = 50.0
            self.damping = 2.0
            self.max_force = 5.0

    def check_safety_limits(self, forces, torques):
        """
        Check if forces and torques are within safe limits
        """
        for force in forces:
            if abs(force) > self.max_force:
                return False, f"Force limit exceeded: {force} > {self.max_force}"

        for torque in torques:
            if abs(torque) > self.max_torque:
                return False, f"Torque limit exceeded: {torque} > {self.max_torque}"

        return True, "Within safety limits"

class ManipulationController:
    def __init__(self):
        self.impedance_controller = ImpedanceController()
        self.grasp_controller = GraspPlanner()
        self.kinematics_solver = HumanoidKinematics()

    def execute_manipulation_task(self, task_description, object_info):
        """
        Execute a manipulation task with proper force control
        """
        # Plan grasp based on object and task
        grasp_plan = self.grasp_controller.plan_grasp(
            object_info['mesh'],
            object_info['properties'],
            task_description['requirements']
        )

        # Calculate approach trajectory
        approach_traj = self.calculate_approach_trajectory(grasp_plan)

        # Execute approach with compliant control
        success = self.execute_approach_with_compliance(approach_traj)

        if not success:
            return False, "Approach failed"

        # Execute grasp with appropriate force control
        grasp_success = self.execute_grasp_with_force_control(grasp_plan)

        if not grasp_success:
            return False, "Grasp failed"

        # Execute manipulation task
        task_success = self.execute_task_with_impedance_control(task_description)

        if not task_success:
            # Release object safely
            self.release_object_safely()
            return False, "Task execution failed"

        # Release object
        release_success = self.release_object_safely()

        return release_success, "Task completed successfully"

    def execute_approach_with_compliance(self, trajectory):
        """
        Execute approach trajectory with compliance control
        """
        for waypoint in trajectory:
            # Calculate desired position
            desired_pos = waypoint['position']

            # Use compliant control to follow trajectory
            adjustment = self.impedance_controller.calculate_impedance_response(
                desired_pos,
                self.get_current_end_effector_position(),
                np.zeros(3),  # Desired force (none during approach)
                self.get_current_force_feedback(),
                0.01  # dt = 10ms
            )

            # Apply position adjustment
            target_pos = desired_pos + adjustment
            self.move_end_effector_to(target_pos)

            # Check for collisions or excessive forces
            current_forces = self.get_current_force_feedback()
            if any(abs(f) > 50 for f in current_forces):  # Collision detection
                return False

        return True

    def execute_grasp_with_force_control(self, grasp_plan):
        """
        Execute grasp with controlled force application
        """
        # Set appropriate impedance for grasping
        self.impedance_controller.adapt_impedance_for_task('grasping')

        # Move fingers to grasp configuration gradually
        initial_config = self.get_current_hand_configuration()
        target_config = grasp_plan['hand_configuration']

        # Interpolate to target configuration with force control
        for t in np.linspace(0, 1, 50):  # 50 steps
            interp_config = self.interpolate_hand_config(initial_config, target_config, t)

            # Apply configuration with force feedback
            self.set_hand_configuration(interp_config)

            # Monitor grasp forces
            grasp_forces = self.get_grasp_force_feedback()
            if not self.validate_grasp_forces(grasp_forces, grasp_plan):
                return False

        # Validate grasp stability
        return self.validate_grasp_stability(grasp_plan)

    def validate_grasp_forces(self, current_forces, grasp_plan):
        """
        Validate that grasp forces are appropriate
        """
        # Check that forces are within expected range for the grasp type
        expected_force_range = self.get_expected_force_range(grasp_plan['grasp_type'])

        for i, force in enumerate(current_forces):
            min_expected, max_expected = expected_force_range[i % len(expected_force_range)]
            if not (min_expected <= abs(force) <= max_expected):
                return False

        return True

    def get_expected_force_range(self, grasp_type):
        """
        Get expected force ranges for different grasp types
        """
        force_ranges = {
            GraspType.POWER_GRASP: [(5, 20), (5, 20), (5, 15), (5, 15)],  # N for each finger
            GraspType.PRECISION_GRASP: [(2, 10), (2, 10)],  # N for thumb and index
            GraspType.CYLINDRICAL_GRASP: [(3, 15), (3, 15), (3, 15), (3, 15)],
            GraspType.SPHERICAL_GRASP: [(4, 18), (4, 18), (4, 18), (4, 18), (4, 18)]
        }

        return force_ranges.get(grasp_type, [(5, 20)] * 5)
```

## ROS 2 Integration for Manipulation

Now let's implement the ROS 2 integration for the manipulation system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, WrenchStamped
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
import numpy as np
import math

class HumanoidManipulationNode(Node):
    def __init__(self):
        super().__init__('humanoid_manipulation_node')

        # Initialize controllers
        self.impedance_controller = ImpedanceController()
        self.grasp_controller = GraspPlanner()
        self.hand_controller = HumanoidHandController()

        # Publishers
        self.left_hand_command_pub = self.create_publisher(JointState, '/left_hand/commands', 10)
        self.right_hand_command_pub = self.create_publisher(JointState, '/right_hand/commands', 10)
        self.arm_trajectory_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.manipulation_status_pub = self.create_publisher(String, '/manipulation/status', 10)
        self.end_effector_pose_pub = self.create_publisher(Pose, '/end_effector/pose', 10)

        # Subscribers
        self.left_hand_state_sub = self.create_subscription(
            JointState, '/left_hand/joint_states', self.left_hand_state_callback, 10
        )
        self.right_hand_state_sub = self.create_subscription(
            JointState, '/right_hand/joint_states', self.right_hand_state_callback, 10
        )
        self.left_ft_sensor_sub = self.create_subscription(
            WrenchStamped, '/left_hand/ft_sensor', self.left_hand_force_callback, 10
        )
        self.right_ft_sensor_sub = self.create_subscription(
            WrenchStamped, '/right_hand/ft_sensor', self.right_hand_force_callback, 10
        )
        self.manipulation_command_sub = self.create_subscription(
            String, '/manipulation/command', self.manipulation_command_callback, 10
        )

        # Timer for manipulation control loop
        self.manipulation_timer = self.create_timer(0.01, self.manipulation_control_loop)  # 100 Hz

        # Robot state
        self.left_hand_state = JointState()
        self.right_hand_state = JointState()
        self.left_hand_forces = WrenchStamped()
        self.right_hand_forces = WrenchStamped()
        self.current_manipulation_task = None
        self.manipulation_active = False

        self.get_logger().info('Humanoid Manipulation Node initialized')

    def left_hand_state_callback(self, msg):
        """Update left hand joint state"""
        self.left_hand_state = msg

    def right_hand_state_callback(self, msg):
        """Update right hand joint state"""
        self.right_hand_state = msg

    def left_hand_force_callback(self, msg):
        """Update left hand force/torque measurements"""
        self.left_hand_forces = msg

    def right_hand_force_callback(self, msg):
        """Update right hand force/torque measurements"""
        self.right_hand_forces = msg

    def manipulation_command_callback(self, msg):
        """Process manipulation commands"""
        command = msg.data
        self.process_manipulation_command(command)

    def process_manipulation_command(self, command):
        """Process manipulation commands"""
        try:
            if command.startswith('grasp '):
                # Extract object info and grasp type
                parts = command.split(' ')
                if len(parts) >= 3:
                    object_type = parts[1]
                    grasp_type = parts[2]

                    # Plan and execute grasp
                    success = self.execute_grasp(object_type, grasp_type)
                    status_msg = String()
                    status_msg.data = f"Grasp {'successful' if success else 'failed'}"
                    self.manipulation_status_pub.publish(status_msg)

            elif command.startswith('move_to '):
                # Extract target position
                parts = command.split(' ')
                if len(parts) >= 4:
                    target_x = float(parts[1])
                    target_y = float(parts[2])
                    target_z = float(parts[3])

                    # Execute movement
                    success = self.move_end_effector_to([target_x, target_y, target_z])
                    status_msg = String()
                    status_msg.data = f"Movement {'successful' if success else 'failed'}"
                    self.manipulation_status_pub.publish(status_msg)

            elif command == 'release':
                # Release grasp
                success = self.release_grasp()
                status_msg = String()
                status_msg.data = f"Release {'successful' if success else 'failed'}"
                self.manipulation_status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing manipulation command: {e}')

    def execute_grasp(self, object_type, grasp_type):
        """Execute grasp on object"""
        try:
            # Get current hand state
            current_config = self.get_current_hand_configuration('left')

            # Plan grasp based on object type
            object_properties = self.get_object_properties(object_type)
            grasp_config = self.hand_controller.generate_grasp_configuration(
                grasp_type, object_properties
            )

            # Validate configuration
            is_valid, validation_msg = self.hand_controller.validate_grasp_configuration(grasp_config)
            if not is_valid:
                self.get_logger().warn(f'Invalid grasp configuration: {validation_msg}')
                return False

            # Execute grasp with force control
            success = self.execute_grasp_with_force_control(grasp_config)

            return success

        except Exception as e:
            self.get_logger().error(f'Error executing grasp: {e}')
            return False

    def execute_grasp_with_force_control(self, target_config):
        """Execute grasp with controlled forces"""
        # Set compliant control parameters for grasping
        self.impedance_controller.adapt_impedance_for_task('grasping')

        # Gradually move to target configuration
        initial_config = self.get_current_hand_configuration('left')
        steps = 50

        for step in range(steps):
            t = step / (steps - 1)

            # Interpolate to target configuration
            current_config = self.interpolate_hand_config(initial_config, target_config, t)

            # Apply configuration
            self.set_hand_configuration('left', current_config)

            # Monitor forces to avoid excessive grip
            left_forces = [abs(f) for f in [
                self.left_hand_forces.wrench.force.x,
                self.left_hand_forces.wrench.force.y,
                self.left_hand_forces.wrench.force.z
            ]]

            max_force = max(left_forces) if left_forces else 0
            if max_force > 30.0:  # Safety limit for grasping
                self.get_logger().warn(f'Excessive force detected: {max_force}N, stopping grasp')
                return False

            # Small delay for smooth motion
            self.get_clock().sleep_for(Duration(seconds=0.01))

        return True

    def move_end_effector_to(self, target_position):
        """Move end effector to target position with impedance control"""
        try:
            # Get current end effector position
            current_pos = self.get_current_end_effector_position('left_arm')

            # Calculate trajectory
            trajectory = self.generate_cartesian_trajectory(current_pos, target_position)

            # Execute trajectory with compliant control
            for waypoint in trajectory:
                # Calculate impedance-adjusted position
                force_feedback = self.get_current_force_feedback('left_arm')
                adjustment = self.impedance_controller.calculate_impedance_response(
                    waypoint,
                    self.get_current_end_effector_position('left_arm'),
                    np.zeros(3),  # Desired force (none during movement)
                    force_feedback,
                    0.01  # dt
                )

                adjusted_target = waypoint + adjustment

                # Publish joint trajectory command
                self.publish_arm_trajectory(adjusted_target)

                # Check for safety limits
                safety_ok, safety_msg = self.impedance_controller.check_safety_limits(
                    [force_feedback], [0, 0, 0]  # Simplified torque check
                )

                if not safety_ok:
                    self.get_logger().warn(f'Safety violation: {safety_msg}')
                    return False

                # Small delay
                self.get_clock().sleep_for(Duration(seconds=0.01))

            return True

        except Exception as e:
            self.get_logger().error(f'Error moving end effector: {e}')
            return False

    def release_grasp(self):
        """Release current grasp"""
        try:
            # Open hand to default position
            open_config = {
                'thumb': [0, 0, 0, 0],
                'index': [0, 0, 0, 0],
                'middle': [0, 0, 0, 0],
                'ring': [0, 0, 0, 0],
                'pinky': [0, 0, 0, 0]
            }

            return self.execute_grasp_with_force_control(open_config)

        except Exception as e:
            self.get_logger().error(f'Error releasing grasp: {e}')
            return False

    def manipulation_control_loop(self):
        """Main manipulation control loop"""
        if self.manipulation_active and self.current_manipulation_task:
            # Execute current manipulation task
            task_result = self.execute_current_task()

            if task_result == 'completed':
                self.manipulation_active = False
                self.current_manipulation_task = None

                # Publish completion status
                status_msg = String()
                status_msg.data = "Task completed successfully"
                self.manipulation_status_publisher.publish(status_msg)

    def get_current_hand_configuration(self, hand_side):
        """Get current hand configuration from joint states"""
        if hand_side == 'left':
            state = self.left_hand_state
        else:
            state = self.right_hand_state

        config = {}
        # Map joint names to finger positions (simplified mapping)
        for i, name in enumerate(state.name):
            if 'thumb' in name:
                if 'thumb' not in config:
                    config['thumb'] = []
                config['thumb'].append(math.degrees(state.position[i]))
            elif 'index' in name:
                if 'index' not in config:
                    config['index'] = []
                config['index'].append(math.degrees(state.position[i]))
            elif 'middle' in name:
                if 'middle' not in config:
                    config['middle'] = []
                config['middle'].append(math.degrees(state.position[i]))
            elif 'ring' in name:
                if 'ring' not in config:
                    config['ring'] = []
                config['ring'].append(math.degrees(state.position[i]))
            elif 'pinky' in name:
                if 'pinky' not in config:
                    config['pinky'] = []
                config['pinky'].append(math.degrees(state.position[i]))

        return config

    def set_hand_configuration(self, hand_side, config):
        """Send hand configuration to robot"""
        # Create joint state message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = []
        joint_msg.position = []

        # Map configuration to joint names
        finger_joints = {
            'thumb': ['thumb_cmc', 'thumb_mcp', 'thumb_ip'],
            'index': ['index_mcp', 'index_pip', 'index_dip'],
            'middle': ['middle_mcp', 'middle_pip', 'middle_dip'],
            'ring': ['ring_mcp', 'ring_pip', 'ring_dip'],
            'pinky': ['pinky_mcp', 'pinky_pip', 'pinky_dip']
        }

        for finger, joints in finger_joints.items():
            if finger in config:
                for j_idx, joint_name in enumerate(joints):
                    if j_idx < len(config[finger]):
                        joint_msg.name.append(f"{hand_side}_{joint_name}")
                        joint_msg.position.append(math.radians(config[finger][j_idx]))

        # Publish command
        if hand_side == 'left':
            self.left_hand_command_publisher.publish(joint_msg)
        else:
            self.right_hand_command_publisher.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)

    node = HumanoidManipulationNode()

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

## Launch File for Manipulation System

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

    # Humanoid manipulation controller
    manipulation_controller = Node(
        package='physical_ai_humanoid_control',
        executable='manipulation_controller_node',
        name='manipulation_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/left_hand/joint_states', '/physical_ai_humanoid/left_hand/joint_states'),
            ('/right_hand/joint_states', '/physical_ai_humanoid/right_hand/joint_states'),
            ('/left_hand/commands', '/physical_ai_humanoid/left_hand/commands'),
            ('/right_hand/commands', '/physical_ai_humanoid/right_hand/commands'),
            ('/left_hand/ft_sensor', '/physical_ai_humanoid/left_hand/ft_sensor'),
            ('/right_hand/ft_sensor', '/physical_ai_humanoid/right_hand/ft_sensor'),
        ],
        output='screen'
    )

    # Grasp planning node
    grasp_planner = Node(
        package='physical_ai_humanoid_control',
        executable='grasp_planner_node',
        name='grasp_planner',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Force control node
    force_controller = Node(
        package='physical_ai_humanoid_control',
        executable='force_controller_node',
        name='force_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_force': 100.0},
            {'compliance_mode': 'adaptive'}
        ],
        output='screen'
    )

    # Hand kinematics node
    hand_kinematics = Node(
        package='physical_ai_humanoid_control',
        executable='hand_kinematics_node',
        name='hand_kinematics',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)

    # Add nodes
    ld.add_action(manipulation_controller)
    ld.add_action(grasp_planner)
    ld.add_action(force_controller)
    ld.add_action(hand_kinematics)

    return ld
```

## Best Practices for Manipulation Systems

### 1. Safety Considerations
- Implement multiple layers of force and position limits
- Use compliant control for human interaction scenarios
- Include collision detection and avoidance
- Design graceful degradation for sensor failures

### 2. Grasp Planning
- Consider object properties (weight, fragility, shape)
- Plan for multiple grasp attempts if first attempt fails
- Use tactile feedback for grasp verification
- Implement grasp stability assessment

### 3. Force Control
- Adapt impedance parameters based on task requirements
- Monitor forces continuously during manipulation
- Implement safety shutdowns for excessive forces
- Use force control for assembly and insertion tasks

### 4. Coordination
- Coordinate arm and hand movements for complex tasks
- Plan whole-body motions for better reach and stability
- Consider balance when manipulating heavy objects
- Use both hands for complex manipulation tasks

## Learning Objectives

By the end of Week 7, you should be able to:
1. Implement dexterous hand control for humanoid robots
2. Plan and execute grasps based on object properties and task requirements
3. Apply force control and impedance control for safe manipulation
4. Integrate manipulation systems with ROS 2 architecture
5. Design manipulation tasks that consider safety and efficiency

## Exercises

### Exercise 1: Basic Hand Control (Beginner)
- **Time**: 60 minutes
- **Objective**: Implement basic hand joint control and simple grasp patterns
- **Steps**: Create hand controller that can execute basic grasp types (open, close, cylindrical)
- **Expected Outcome**: Working hand control system that can execute basic grasps

### Exercise 2: Grasp Planning (Intermediate)
- **Time**: 90 minutes
- **Objective**: Implement grasp planning based on object properties
- **Steps**: Create system that analyzes object shape and plans appropriate grasp
- **Expected Outcome**: Grasp planner that selects appropriate grasp type for different objects

### Exercise 3: Force-Controlled Manipulation (Advanced)
- **Time**: 120 minutes
- **Objective**: Implement compliant manipulation with force control
- **Steps**: Create system that manipulates objects with controlled forces
- **Expected Outcome**: Manipulation system that can handle objects safely with appropriate compliance

## Summary

Week 7 covered manipulation and grasping systems for humanoid robots, including dexterous hand control, grasp planning, force control, and ROS 2 integration. You learned to implement mathematical models for hand kinematics, design grasp planning algorithms, and create compliant control systems for safe manipulation. These skills enable humanoid robots to interact effectively with objects in human environments. In Week 8, we'll explore human-robot interaction and conversational AI integration.