# Copyright 2020 Louise Poubel
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

"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_dolly_ignition = get_package_share_directory('dolly_ignition')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
    )

    # Spawn dolly
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'dolly',
                    '-x', '5.0',
                    '-z', '0.46',
                    '-Y', '1.57',
                    '-file', os.path.join(pkg_dolly_ignition, 'models', 'dolly_ignition',
                                          'model.sdf')],
                 output='screen')

    # Follow node
    follow = Node(
        package='dolly_follow',
        executable='dolly_follow',
        output='screen',
        remappings=[
            ('cmd_vel', '/dolly/cmd_vel'),
            ('laser_scan', '/dolly/laser_scan')
        ]
    )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/dolly/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/dolly/laser_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   '/dolly/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dolly_ignition, 'rviz', 'dolly_ignition.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'ign_args',
          default_value=[os.path.join(pkg_dolly_ignition, 'worlds', 'station.sdf') +
                         ' -v 2 --gui-config ' +
                         os.path.join(pkg_dolly_ignition, 'ign', 'gui.config'), ''],
          description='Ignition Gazebo arguments'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        spawn,
        follow,
        bridge,
        rviz
    ])
