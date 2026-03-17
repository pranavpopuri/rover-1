#!/usr/bin/env python3
"""
Launch file for Rover2 sensor nodes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_sensors = get_package_share_directory('rover2_sensors')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # INA219 power monitor node
    ina219_node = Node(
        package='rover2_sensors',
        executable='ina219_node',
        name='ina219_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'i2c_bus': 1,
            'i2c_address': 0x40,
            'publish_rate': 10.0,
            'low_voltage_threshold': 13.0,
            'critical_voltage_threshold': 12.0,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        ina219_node,
    ])
