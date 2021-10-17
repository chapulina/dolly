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

import ament_index_python

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers

from launch_ros.actions import Node


def generate_test_description():
    follow_test_path = os.path.join(
        ament_index_python.get_package_prefix('dolly_ignition'),
        'lib/dolly_ignition',
        'follow_TEST'
    )

    test_process = Node(
        package='dolly_ignition',
        executable='follow_TEST',
        output='screen'
    )

    return launch.LaunchDescription([
        test_process,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()


class DollyFollowTest(unittest.TestCase):

    def test_termination(self, test_process, proc_info):
        proc_info.assertWaitForShutdown(process=test_process, timeout=200)


@launch_testing.post_shutdown_test()
class DollyFollowTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, test_process, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            test_process
        )
