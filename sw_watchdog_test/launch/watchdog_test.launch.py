#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
"""Launch a Watchdog, Heartbeat and example Nodes."""

import os

import launch  # type: ignore
from ament_index_python.packages import (  # type: ignore # noqa: I100
    get_package_share_directory,
)
from launch.actions import IncludeLaunchDescription  # type: ignore
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart  # type: ignore
from launch.launch_description_sources import (  # type: ignore
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """Generate a launch configuration for this test."""
    watchdog_a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('sw_watchdog_test'), 'launch'
                ),
                '/watchdog_namespaced_a.launch.py',
            ]
        )
    )
    heartbeat_a = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('sw_watchdog_test'), 'launch'
                ),
                '/heartbeat_namespaced_a.launch.py',
            ]
        )
    )
    watchdog_b = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('sw_watchdog_test'), 'launch'
                ),
                '/watchdog_namespaced_b.launch.py',
            ]
        )
    )
    heartbeat_b = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('sw_watchdog_test'), 'launch'
                ),
                '/heartbeat_namespaced_b.launch.py',
            ]
        )
    )
    watchdog_monitor = Node(
        package='sw_watchdog',
        executable='watchdog_monitor',
        namespace='my_namespace',
        name='watchdog_monitor',
        parameters=[{'nodes_list': ['/node_a/watchdog', '/node_b/watchdog']}],
    )

    return launch.LaunchDescription(
        [
            watchdog_monitor,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=watchdog_monitor,
                    on_start=[
                        LogInfo(msg='Watchdog Monitor has started...'),
                        watchdog_a,
                        heartbeat_a,
                        watchdog_b,
                        heartbeat_b,
                    ],
                )
            ),
        ]
    )
