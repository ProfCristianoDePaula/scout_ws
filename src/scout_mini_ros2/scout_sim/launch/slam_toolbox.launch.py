#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_scout_sim    = get_package_share_directory('scout_sim')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(pkg_scout_sim, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time':     'true',
            'slam_params_file': slam_params,
        }.items(),
    )

    return LaunchDescription([
        slam_toolbox_launch,
    ])
