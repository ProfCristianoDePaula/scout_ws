#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scout_control',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            output='screen',
        ),
    ])