#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
"""Launch a node_a and a heartbeat in a component container."""

import launch  # type: ignore
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> launch.LaunchDescription:
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='node_a_container',
        namespace='node_a',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Example to add node 'node_a' to be watched by Watchdog
            ComposableNode(
                package='sw_watchdog_test',
                plugin='sw_watchdog_test::TestNode',
                name='node_a',
                namespace='node_a',
            ),
            ComposableNode(
                package='sw_watchdog',
                plugin='sw_watchdog::SimpleHeartbeat',
                namespace='node_a',
                name='node_a_heartbeat',
                parameters=[{'period': 200}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
