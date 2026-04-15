#!/usr/bin/env python3
"""
scout_sim.launch.py
===================
Launches the Scout Mini simulation in Gazebo Harmonic with full ROS2 integration.

What this launch file does:
  1. Starts Gazebo Harmonic (gz sim) with scout_world.sdf
  2. Spawns the Scout Mini robot from URDF/Xacro
  3. Starts robot_state_publisher (URDF → /tf)
  4. Starts ros_gz_bridge (Gazebo topics ↔ ROS2 topics)

Usage:
  ros2 launch scout_sim scout_sim.launch.py
  ros2 launch scout_sim scout_sim.launch.py use_sim_time:=true
  ros2 launch scout_sim scout_sim.launch.py headless:=true

Authors: Milad, AMR Course UFSCar 2026
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro



def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────
    pkg_scout_sim = get_package_share_directory('scout_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path   = os.path.join(pkg_scout_sim, 'urdf', 'scout_mini.urdf.xacro')
    world_path = os.path.join(pkg_scout_sim, 'worlds', 'dc_with_obstacles.sdf')
    bridge_config = os.path.join(pkg_scout_sim, 'config', 'ros_gz_bridge.yaml')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_scout_sim)
    )

    # ── Process URDF/Xacro → robot_description string ──────────────────
    robot_description = xacro.process_file(urdf_path).toxml()

    # ── Launch arguments ────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI (server only)')

    render_engine_arg = DeclareLaunchArgument(
        'render_engine', default_value='ogre2',
        description='Gazebo render engine: ogre2 (default) or ogre (VirtualBox fallback)')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    headless      = LaunchConfiguration('headless')
    render_engine = LaunchConfiguration('render_engine')

    # ── Gazebo Harmonic launch ──────────────────────────────────────────
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],   # -r = start running immediately
        }.items(),
    )

    # ── Robot State Publisher (publishes /robot_description + static TF) -
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Spawn robot in Gazebo ───────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_scout_mini',
        output='screen',
        arguments=[
            '-name', 'scout_mini',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',   # spawn slightly above ground to avoid collision on start
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    # ── ROS–Gazebo bridge ───────────────────────────────────────────────
    # Bridges Gazebo topics to ROS2 and vice versa using the config YAML.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            #'config_file': bridge_config,
            #'use_sim_time': use_sim_time,
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.Imu',
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        headless_arg,
        render_engine_arg,

        gz_sim_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
    ])
