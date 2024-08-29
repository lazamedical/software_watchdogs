#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
"""Launch a watchdog lifecycle node under the namespace."""

import launch  # type: ignore
from launch.actions import LogInfo  # type: ignore
from launch.actions import RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description() -> launch.LaunchDescription:
    """Generate a launch configuration for this module."""
    set_tty_launch_config_action = launch.actions.SetLaunchConfiguration(
        'emulate_tty', 'True'
    )
    watchdog_node = LifecycleNode(
        package='sw_watchdog',
        executable='simple_watchdog',
        namespace='node_b',
        name='watchdog',
        output='screen',
        arguments=['320', '--publish', '--activate'],
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
            watchdog_inactive_handler,
        ]
    )
