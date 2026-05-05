#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────
    pkg_scout_sim  = get_package_share_directory('scout_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path     = os.path.join(pkg_scout_sim, 'urdf',   'scout_mini.urdf.xacro')
    world_path    = os.path.join(pkg_scout_sim, 'worlds', 'maze_class7.sdf')
    bridge_config = os.path.join(pkg_scout_sim, 'config', 'ros_gz_bridge.yaml')

    # ── Set GZ_SIM_RESOURCE_PATH so Gazebo can find meshes ─────────────
    # Without this the wheel/body .dae files are not located and the robot
    # spawns without visual meshes (physics/sensors still work, but it's
    # good practice to set it explicitly).
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_scout_sim),
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

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Gazebo Harmonic ─────────────────────────────────────────────────
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path],
        }.items(),
    )

    # ── Robot State Publisher ───────────────────────────────────────────
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
    # Using '-string' (pre-processed URDF) instead of '-topic' to avoid
    # a QoS mismatch: robot_state_publisher uses transient_local durability
    # but ros_gz_sim create subscribes volatile, so it misses the message.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_scout_mini',
        output='screen',
        arguments=[
            '-name', 'scout_mini',
            '-string', robot_description,
            '-x', '-8.0',
            '-y',  '8.0',
            '-z',  '0.2',   # slightly above ground to avoid collision on start
            '-R',  '0.0',
            '-P',  '0.0',
            '-Y',  '0.0',   # facing east (yaw = 0)
        ],
    )

    # ── ROS–Gazebo bridge ───────────────────────────────────────────────
    # IMPORTANT: use config_file ONLY — do NOT also pass an arguments list
    # with the same topics.  Having both causes every topic to be bridged
    # twice: /cmd_vel would send double velocity commands, /tf would produce
    # duplicate transforms, etc.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        set_gz_resource_path,
        use_sim_time_arg,
        headless_arg,
        render_engine_arg,

        gz_sim_launch,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
    ])
