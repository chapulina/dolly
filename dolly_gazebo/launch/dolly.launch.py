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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable
from launch import LaunchContext

def setenv(lc):
    with open("/usr/share/gazebo/setup.sh") as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip().split()
            if len(line) > 1 and line[0] == "export":
                line = line[1].split('=')
                ev = ""
                if len(line) > 1:
                    ev = line[0]
                    curr = "${"+ev+"}"
                    prepend = line[1].endswith(curr)
                    val = line[1].replace(curr, "")
                    if os.environ.get(ev):
                        if not prepend:
                            if val[0] != os.pathsep:
                                val = os.pathsep + val
                            SetEnvironmentVariable(ev, [EnvironmentVariable(ev), val]).visit(lc)
                        else:
                            if val[-1] != os.pathsep:
                                val += os.pathsep
                            SetEnvironmentVariable(ev, [val, EnvironmentVariable(ev)]).visit(lc)
                    else:
                        SetEnvironmentVariable(ev, val).visit(lc)
                    #  print(ev+"======")
                    #  print(os.environ.get(ev))

def generate_launch_description():

    lc = LaunchContext()

    setenv(lc)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_dolly_gazebo = get_package_share_directory('dolly_gazebo')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

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

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dolly_gazebo, 'rviz', 'dolly_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_dolly_gazebo, 'worlds', 'dolly_empty.world'), ''],
          description='SDF world file'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gazebo,
        follow,
        rviz
    ])
