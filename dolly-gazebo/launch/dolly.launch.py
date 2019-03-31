# Copyright 2019 Louise Poubel
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

from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    gzserver_exe = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             launch.substitutions.LaunchConfiguration('world')],
        output='screen'
    )
    gzclient_exe = launch.actions.ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    follow = launch_ros.actions.Node(
        package='dolly-follow',
        node_executable='dolly-follow',
        output='screen',
        remappings=[
            ('cmd_vel', '/dolly/cmd_vel'),
            ('laser_scan', '/dolly/laser_scan')
        ]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=['worlds/empty.world', ''],
          description='Gazebo world file'),
        gzserver_exe,
        gzclient_exe,
        follow
    ])
