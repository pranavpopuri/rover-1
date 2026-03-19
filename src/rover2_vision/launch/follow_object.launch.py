#!/usr/bin/env python3
"""
Launch object detection and following.

Usage:
    ros2 launch rover2_vision follow_object.launch.py
    ros2 launch rover2_vision follow_object.launch.py target_class:=cup
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vision = get_package_share_directory('rover2_vision')
    config_file = os.path.join(pkg_vision, 'config', 'vision.yaml')

    target_class = LaunchConfiguration('target_class', default='bottle')

    # USB camera
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 15.0,
        }],
    )

    # Object detector
    detector_node = Node(
        package='rover2_vision',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[
            config_file,
            {'target_class': target_class},
        ],
    )

    # Object follower
    follower_node = Node(
        package='rover2_vision',
        executable='follower_node',
        name='follower_node',
        output='screen',
        parameters=[config_file],
    )

    # Web dashboard
    dashboard_node = Node(
        package='rover2_vision',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        parameters=[{'port': 8080}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_class',
            default_value='bottle',
            description='Object class to detect and follow (COCO class name)'
        ),
        camera_node,
        detector_node,
        follower_node,
        dashboard_node,
    ])
