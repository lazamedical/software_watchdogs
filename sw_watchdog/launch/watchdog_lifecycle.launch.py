#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
# Copyright (c) 2020 Mapless AI, Inc.
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

import launch  # type: ignore
from launch.actions import LogInfo  # type: ignore
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    set_tty_launch_config_action = launch.actions.SetLaunchConfiguration('emulate_tty', 'True')
    watchdog_node = LifecycleNode(
        package='sw_watchdog',
        executable='simple_watchdog',
        namespace='',
        name='simple_watchdog',
        output='screen',
        arguments=['220', '--publish', '--activate']
        # arguments=['__log_level:=debug']
    )
    # When the watchdog reaches the 'inactive' state, log a message
    watchdog_inactive_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=watchdog_node,
            goal_state='inactive',
            entities=[
                # Log
                LogInfo(msg="Watchdog transitioned to 'INACTIVE' state."),
            ],
        )
    )
    return launch.LaunchDescription(
        [
            set_tty_launch_config_action,
            watchdog_node,
            watchdog_inactive_handler
        ]
    )
