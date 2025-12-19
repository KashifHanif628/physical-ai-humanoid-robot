"""
Visual SLAM Launch Configuration for Isaac ROS

This script demonstrates how to configure and launch Isaac ROS Visual SLAM
with proper parameters for hardware acceleration.
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate the launch description for Isaac ROS Visual SLAM
    """
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'enable_fisheye',
            default_value='False',
            description='Enable fisheye camera for visual slam'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='False',
            description='Enable depth camera for visual slam'
        ),
        DeclareLaunchArgument(
            'rectified_images',
            default_value='False',
            description='Use rectified images for visual slam'
        ),
    ]

    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_fisheye = LaunchConfiguration('enable_fisheye')
    enable_depth = LaunchConfiguration('enable_depth')
    rectified_images = LaunchConfiguration('rectified_images')

    # Visual SLAM node configuration
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_fisheye': enable_fisheye,
            'enable_depth': enable_depth,
            'rectified_images': rectified_images,
            'enable_occupancy_map': True,
            'occupancy_map_depth': 20,
            'min_num_points_per_keyframe': 100,
            'min_keyframe_travel_meters': 0.5,
            'min_keyframe_rotation_radians': 0.1,
            'publish_metrics': True,
            'publish_localization_result': True,
            'publish_pose_graph': True,
        }],
        remappings=[
            ('/visual_slam/image', '/camera/image_raw'),
            ('/visual_slam/camera_info', '/camera/camera_info'),
            ('/visual_slam/imu', '/imu/data'),
        ],
        output='screen'
    )

    # Occupancy grid node configuration
    occupancy_grid_node = Node(
        package='isaac_ros_visual_slam',
        executable='occupancy_grid_node',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('/occupancy_grid', '/visual_slam/occupancy_grid'),
        ],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        *launch_args,
        visual_slam_node,
        occupancy_grid_node,
    ])


def main():
    """
    Main function to demonstrate visual SLAM launch configuration
    """
    print("Isaac ROS Visual SLAM Launch Configuration")
    print("===========================================")
    print("This configuration sets up Isaac ROS Visual SLAM with:")
    print("- Hardware acceleration parameters")
    print("- Camera and IMU integration")
    print("- Occupancy grid mapping")
    print("- Performance metrics publishing")
    print("")
    print("To launch with default parameters:")
    print("ros2 launch visual_slam_launch.py")
    print("")
    print("To launch with simulation time:")
    print("ros2 launch visual_slam_launch.py use_sim_time:=true")
    print("")
    print("To enable fisheye camera:")
    print("ros2 launch visual_slam_launch.py enable_fisheye:=true")


if __name__ == '__main__':
    main()