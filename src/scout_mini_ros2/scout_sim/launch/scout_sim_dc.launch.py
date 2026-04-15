#!/usr/bin/env python3
"""
scout_sim_dc.launch.py
======================
Launches the Scout Mini Challenge 2 simulation in Gazebo Harmonic.

What this launch file does:
  1. Starts Gazebo Harmonic server (-s) with dc_with_obstacles.sdf
  2. Starts Gazebo GUI (-g) separately (so GUI crash doesn't kill the server)
  3. Spawns the Scout Mini robot from URDF/Xacro
  4. Starts robot_state_publisher (URDF → /tf)
  5. Starts ros_gz_bridge (Gazebo topics ↔ ROS2 topics)
  6. Starts challenge2_navigator (autonomous obstacle avoidance)

Usage:
  ros2 launch scout_sim scout_sim_dc.launch.py
  ros2 launch scout_sim scout_sim_dc.launch.py headless:=true
  ros2 launch scout_sim scout_sim_dc.launch.py start_autonomy:=false

Authors: Milad, AMR Course UFSCar 2026
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # ── Package paths ───────────────────────────────────────────────────
    pkg_scout_sim = get_package_share_directory('scout_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_path  = os.path.join(pkg_scout_sim, 'urdf', 'scout_mini.urdf.xacro')
    world_path = os.path.join(pkg_scout_sim, 'worlds', 'dc_with_obstacles.sdf')

    # Required for Gazebo to locate meshes/models from this package
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

    start_autonomy_arg = DeclareLaunchArgument(
        'start_autonomy', default_value='false',
        description='Start autonomous challenge2 navigator automatically (use Terminal 2 instead)')

    render_engine_arg = DeclareLaunchArgument(
        'render_engine', default_value='ogre2',
        description='Render engine for Gazebo sensors: ogre2 or ogre')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    headless      = LaunchConfiguration('headless')
    start_autonomy = LaunchConfiguration('start_autonomy')

    # ── Gazebo SERVER (always runs, server-only mode -s) ────────────────
    # Running with -s so the server is an independent process.
    # This prevents the GUI crash (snap/libpthread conflict) from
    # killing the physics simulation.
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world_path],
        }.items(),
    )

    # ── Gazebo GUI (separate process, skipped in headless mode) ─────────
    # Connects to the already-running server via gz transport.
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=UnlessCondition(headless),
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
            '-z', '0.2',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.14',
        ],
    )

    # ── ROS–Gazebo bridge ───────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )

    # ── Autonomous navigator (delayed 4 s to wait for simulation) ───────
    auto_navigator = TimerAction(
        period=4.0,
        condition=IfCondition(start_autonomy),
        actions=[
            Node(
                package='scout_control',
                executable='challenge2_navigator',
                name='challenge2_navigator',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        set_gz_resource_path,

        use_sim_time_arg,
        headless_arg,
        start_autonomy_arg,
        render_engine_arg,

        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        auto_navigator,
    ])
