---
title: "Navigation with Nav2"
sidebar_label: "Chapter 3: Nav2 Navigation"
description: "Learn about Nav2 stack for humanoid robot navigation"
keywords: ["nav2", "navigation", "path planning", "humanoid robots", "localization"]
---

# Navigation with Nav2

## Introduction

Navigation2 (Nav2) is the standard navigation framework for ROS 2, providing a comprehensive solution for robot navigation. This chapter covers the Nav2 stack with a focus on humanoid-specific navigation requirements, building on the perception knowledge from Chapters 1 and 2.

## Nav2 Stack Overview

The Nav2 stack provides a complete navigation solution with:

- **Global Planner**: Plans the overall path from start to goal
- **Local Planner**: Handles dynamic obstacle avoidance and path following
- **Controller**: Manages robot motion along the planned path
- **Costmap**: Represents obstacles and navigation costs in the environment
- **Recovery Behaviors**: Handles navigation failures and recovery

## Localization and Mapping

Nav2 provides robust localization and mapping capabilities:

- **AMCL (Adaptive Monte Carlo Localization)**: Probabilistic localization
- **SLAM Toolbox**: Simultaneous localization and mapping
- **Map Server**: Manages static and costmaps
- **Transform Management**: Handles coordinate frame transformations

### Example: Launching Navigation with Isaac-specific Configurations

```bash
# Launch Nav2 with Isaac-specific configurations
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/humanoid_nav2_config.yaml
```

## Path Planning for Humanoid Robots

Humanoid navigation requires special considerations due to:

- **Bipedal Gait Patterns**: Walking patterns specific to two-legged locomotion
- **Balance and Stability**: Maintaining balance during navigation
- **Step Planning**: Planning individual steps rather than continuous paths
- **Dynamic Obstacle Avoidance**: Safe navigation around other bipedal agents

### Humanoid-Specific Navigation Challenges

1. **Stability Constraints**: Path planning must consider balance and stability
2. **Step Sequencing**: Planning safe sequences of foot placements
3. **Terrain Analysis**: Evaluating terrain for safe bipedal traversal
4. **Gait Adaptation**: Adjusting walking patterns based on terrain

## Example: Biped-Safe Navigation Pipeline

```bash
# Send a navigation goal for humanoid robot
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Nav2 Configuration for Humanoids

Configuration files for humanoid navigation include special parameters:

- **Planner Constraints**: Humanoid-specific path planning parameters
- **Controller Parameters**: Bipedal motion control settings
- **Safety Margins**: Additional safety buffers for humanoid stability
- **Recovery Behaviors**: Humanoid-appropriate recovery actions

### Example Configuration (humanoid_nav2_config.yaml)

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path where the BT XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_are_error_positions_close_bt_node
    - nav2_would_a_controller_recovery_help_condition_bt_node
    - nav2_am_i_oscillating_condition_bt_node
    - nav2_am_i_moving_slowly_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_close_to_goal_bt_node
    - nav2_follow_path_nodes_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      # Additional humanoid-specific parameters
      max_linear_speed: 0.5  # Slower for stability
      max_angular_speed: 0.6
      min_linear_speed: 0.1
      min_angular_speed: 0.1
      approach_tolerance: 0.1
      goal_tolerance: 0.25
      xv_samples: 7
      yv_samples: 0
      thetav_samples: 15
      controller_frequency: 20.0
</pre>

## Integration with Isaac Tools

Nav2 integrates with Isaac tools for comprehensive AI brain functionality:

- **Simulation Integration**: Use Isaac Sim for navigation testing
- **Perception Integration**: Combine with Isaac ROS perception data
- **Hardware Acceleration**: Leverage Isaac hardware acceleration for navigation algorithms

## Troubleshooting

For common issues, refer to the [troubleshooting guide](./common/troubleshooting-guide.md).

## Summary

Nav2 provides a comprehensive navigation solution for humanoid robots with special considerations for bipedal locomotion. The stack integrates with Isaac tools for a complete AI brain solution, combining perception from Chapter 2 with navigation capabilities. This completes the Isaac AI Brain module, providing students with the knowledge to implement complete autonomous humanoid robot systems.