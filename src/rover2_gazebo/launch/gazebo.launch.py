#!/usr/bin/env python3
"""
Launch file for spawning rover2 in Gazebo simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rover2_description = get_package_share_directory('rover2_description')
    pkg_rover2_gazebo = get_package_share_directory('rover2_gazebo')

    # Paths
    urdf_file = os.path.join(pkg_rover2_description, 'urdf', 'rover2_sim.urdf.xacro')
    world_file = os.path.join(pkg_rover2_gazebo, 'worlds', 'rover2_world.sdf')
    rviz_config = os.path.join(pkg_rover2_description, 'rviz', 'rover2.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # Process xacro to get robot description
    robot_description = Command(['xacro ', urdf_file])

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items()
    )

    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

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

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rover2',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rover2',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ]
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Path to world file'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run in headless mode'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Initial X position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial Y position'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.1',
            description='Initial Z position'
        ),

        # Launch nodes
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])
