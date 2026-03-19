#!/usr/bin/env python3
"""
Mock hardware launch file for Rover2.
Uses ros2_control mock_components/GenericSystem to test the full control
pipeline (controllers, TF, odometry) without real hardware or a simulator.

Usage:
    ros2 launch rover2_bringup simulation.launch.py
    ros2 launch rover2_bringup simulation.launch.py rviz:=true
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('rover2_description')
    pkg_base = get_package_share_directory('rover2_base')

    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'rover2_mock.urdf.xacro')
    controller_config = os.path.join(pkg_base, 'config', 'rover2_controllers.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'rover2.rviz')

    # Process xacro
    robot_description = xacro.process_file(urdf_file).toxml()

    # Launch arguments
    rviz = LaunchConfiguration('rviz', default='false')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # ros2_control controller manager
    # Controllers are auto-loaded and activated from the YAML config
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config
        ],
        output='screen',
    )

    # Spawn controllers (delayed to let controller_manager initialize)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '-c', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '-c', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz'
        ),

        # Nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_diff_drive,
        rviz_node,
    ])
