#!/usr/bin/env python3
"""
Main launch file for Rover2 on real hardware (Raspberry Pi 5).

Launches:
  - robot_state_publisher (URDF)
  - ros2_control controller_manager
  - diff_drive_controller
  - joint_state_broadcaster
  - INA219 power monitor (optional)

Usage:
  ros2 launch rover2_bringup robot.launch.py
  ros2 launch rover2_bringup robot.launch.py use_power_monitor:=false
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('rover2_description')
    pkg_base = get_package_share_directory('rover2_base')
    pkg_sensors = get_package_share_directory('rover2_sensors')

    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'rover2_hardware.urdf.xacro')
    controller_config = os.path.join(pkg_base, 'config', 'rover2_controllers.yaml')

    # Process xacro
    robot_description = xacro.process_file(urdf_file).toxml()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_power_monitor = LaunchConfiguration('use_power_monitor', default='true')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # ros2_control controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config
        ],
        output='screen',
    )

    # Spawn diff_drive_controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '-c', '/controller_manager',
                   '-p', controller_config,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '-c', '/controller_manager',
                   '-p', controller_config,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    # Delay diff_drive_controller until joint_state_broadcaster is ready
    delay_diff_drive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # INA219 power monitor (optional)
    power_monitor = Node(
        package='rover2_sensors',
        executable='ina219_node',
        name='ina219_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'i2c_bus': 1,
            'i2c_address': 0x40,
            'publish_rate': 10.0,
        }],
        condition=IfCondition(use_power_monitor)
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_power_monitor',
            default_value='true',
            description='Launch INA219 power monitor'
        ),

        # Nodes
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_diff_drive,
        power_monitor,
    ])
