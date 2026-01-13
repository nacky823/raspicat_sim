# Copyright 2023 RT Corporation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory('raspicat_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    x_pose = LaunchConfiguration('x_pose', default='6.5')
    y_pose = LaunchConfiguration('y_pose', default='8.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='3.1416')
    world = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('raspicat_gazebo'),
        'worlds',
        'iscas_museum_with_pedestrians_dits.world')
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Set to "false" to run headless.'
    )

    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('raspicat_gazebo'),
            'worlds',
            'iscas_museum_with_pedestrians_dits.world'),
        description='world configuration file path'
    )

    declare_x_position = DeclareLaunchArgument(
        'x_pose', default_value='6.5',
        description='x position of robot')

    declare_y_position = DeclareLaunchArgument(
        'y_pose', default_value='8.0',
        description='y position of robot')
    declare_yaw_position = DeclareLaunchArgument(
        'yaw_pose', default_value='3.1416',
        description='yaw position of robot (rad)')

    raspicat_with_emptyworld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'raspicat_with_emptyworld.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose,
            'world': world,
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_x_position)
    ld.add_action(declare_y_position)
    ld.add_action(declare_yaw_position)

    ld.add_action(raspicat_with_emptyworld)

    return ld
