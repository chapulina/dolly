# Copyright 2021 Louise Poubel
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
import unittest
import launch_testing

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_test_description():

    # Test fixture
    gazebo_test_fixture = Node(
        package='dolly_tests',
        executable='follow_ignition_TEST',
        output='screen'
    )

    # Spawn dolly
    pkg_dolly_ignition = get_package_share_directory('dolly_ignition')
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'dolly',
                    '-z', '0.225',
                    '-file', os.path.join(pkg_dolly_ignition, 'models', 'dolly_ignition',
                                          'model.sdf')],
                 output='screen')

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/dolly/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/dolly/laser_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        output='screen'
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


    return launch.LaunchDescription([
        gazebo_test_fixture,
        spawn,
        bridge,
        follow,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class DollyFollowTest(unittest.TestCase):

    def test_termination(self, gazebo_test_fixture, proc_info):
        proc_info.assertWaitForShutdown(process=gazebo_test_fixture, timeout=200)


@launch_testing.post_shutdown_test()
class DollyFollowTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, gazebo_test_fixture, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            gazebo_test_fixture
        )
