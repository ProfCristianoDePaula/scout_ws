# Copyright (c) 2021 Weston Robot Pte. Ltd.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Weston Robot Pte. Ltd. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='can0',
        description='CAN bus name, e.g. can0')

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame id')

    base_link_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame id')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic_name',
        default_value='odom',
        description='Odometry topic name')

    is_scout_mini_arg = DeclareLaunchArgument(
        'is_scout_mini',
        default_value='false',
        description='Scout mini model')

    is_omni_wheel_arg = DeclareLaunchArgument(
        'is_omni_wheel',
        default_value='false',
        description='Scout mini omni-wheel model')

    simulated_robot_arg = DeclareLaunchArgument(
        'simulated_robot',
        default_value='false',
        description='Whether running with simulator')

    sim_control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50',
        description='Simulation control loop update rate')

    scout_base_node = launch_ros.actions.Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
            'port_name': launch.substitutions.LaunchConfiguration('port_name'),
            'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
            'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
            'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
            'is_scout_mini': launch.substitutions.LaunchConfiguration('is_scout_mini'),
            'is_omni_wheel': launch.substitutions.LaunchConfiguration('is_omni_wheel'),
            'simulated_robot': launch.substitutions.LaunchConfiguration('simulated_robot'),
            'control_rate': launch.substitutions.LaunchConfiguration('control_rate'),
        }])

    return LaunchDescription([
        use_sim_time_arg,
        port_name_arg,
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        is_scout_mini_arg,
        is_omni_wheel_arg,
        simulated_robot_arg,
        sim_control_rate_arg,
        scout_base_node
    ])
