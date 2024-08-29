#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
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
"""Launch a watchdog status monitor node."""

import launch  # type: ignore
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """Generate a launch configuration for this module."""
    watchdog_monitor = Node(
        package='sw_watchdog',
        executable='watchdog_monitor',
        name='watchdog_monitor',
        parameters=[{'nodes_list': ['/node_a/watchdog', '/node_b/watchdog']}],
    )

    return launch.LaunchDescription([watchdog_monitor])
