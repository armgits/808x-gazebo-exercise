#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                                                      ExecuteProcess)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    record_bag_value = LaunchConfiguration('record_bag')
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='False',
        description="Record messages on all topics to a rosbag?"
    )

    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    turtlebot3_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py')
        ),
        launch_arguments={'x_pose': '-1.0', 'y_pose': '3.0'}.items()
        )

    record_bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', "/camera/(.*)", '-o', 'output_bag'],
        output='screen',
        condition=IfCondition(PythonExpression([record_bag_value]))
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(record_bag_arg)
    ld.add_action(turtlebot3_launch_cmd)
    ld.add_action(record_bag_process)

    return ld
