#!/usr/bin/env python3
"""
Main simulation launch file for Rover2.
Launches Gazebo, robot state publisher, and optionally RViz and teleop.

Usage:
    ros2 launch rover2_bringup simulation.launch.py
    ros2 launch rover2_bringup simulation.launch.py rviz:=false
    ros2 launch rover2_bringup simulation.launch.py world:=/path/to/custom.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directories
    pkg_rover2_gazebo = get_package_share_directory('rover2_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    teleop = LaunchConfiguration('teleop', default='false')

    # Default world file
    default_world = os.path.join(pkg_rover2_gazebo, 'worlds', 'rover2_world.sdf')

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover2_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'gui': gui,
            'rviz': rviz,
        }.items()
    )

    # Optional teleop (run in separate terminal for better UX)
    teleop_instructions = ExecuteProcess(
        cmd=['echo', '\n\n=== To control the robot, run in another terminal: ===\nros2 run teleop_twist_keyboard teleop_twist_keyboard\n'],
        output='screen',
        condition=IfCondition(teleop)
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to Gazebo world file'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz'
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Print teleop instructions'
        ),

        # Launch
        gazebo_launch,
        teleop_instructions,
    ])
