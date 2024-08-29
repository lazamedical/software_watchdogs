#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
"""Launch analyzer loader with parameters from yaml."""

from launch import LaunchDescription  # type: ignore # noqa: I00
from launch.substitutions import PathJoinSubstitution  # type: ignore
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    diag_config_path = PathJoinSubstitution(
        [
            FindPackageShare('navecho_diagnostics'),
            'config',
            'configuration.yaml',
        ]
    )
    """Generate the launch description."""
    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[diag_config_path],
    )
    return LaunchDescription(
        [
            aggregator,
        ]
    )
