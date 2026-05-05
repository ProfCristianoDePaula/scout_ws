#!/usr/bin/env python3
"""
scout_sim_maze_nav2.launch.py
=============================
Launches the full Week 7 Exercise 2 stack:

    Gazebo + Scout Mini + ROS↔Gazebo bridge      (from scout_sim_maze_2.launch.py)
    nav2_map_server         — serves the saved map
    nav2_amcl               — particle-filter localization
    nav2_planner_server     — global planner (NavFn)
    nav2_controller_server  — local controller (DWB)
    nav2_behavior_server    — recoveries (spin, backup, wait)
    nav2_bt_navigator       — behavior tree navigation
    nav2_lifecycle_manager  — manages lifecycle of all the above

Author: Milad (TA), Autonomous Mobile Robots Course, UFSCar
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_scout_sim    = get_package_share_directory('scout_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    map_file_default    = os.path.join(pkg_scout_sim, 'maps',   'maze_class7_map.yaml')
    nav2_params_default = os.path.join(pkg_scout_sim, 'config', 'nav2_params.yaml')

    # ── Launch arguments ────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_default,
        description='Full path to the .yaml map file to load')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_default,
        description='Full path to the Nav2 ROS 2 parameters file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock')

    # ── 1. Simulation: Gazebo + Scout Mini + bridge ────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scout_sim, 'launch', 'scout_sim_maze_2.launch.py')
        ),
    )

    # ── 2. Nav2 stack: map_server + AMCL + planner + controller + BT ──
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file':  LaunchConfiguration('params_file'),
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,

        sim_launch,
        nav2_launch,
    ])
