#
# Copyright (c) 2024 by Laza Medical.
# All Rights Reserved.
#
"""Watchdog launch test."""
import os
import subprocess  # nosec
import unittest

import pytest
from ament_index_python.packages import (  # type: ignore # noqa: I100
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription  # type: ignore
from launch.launch_description_sources import (  # type: ignore
    PythonLaunchDescriptionSource,
)
from launch_testing.actions import ReadyToTest  # type: ignore
from launch_testing_ros import WaitForTopics  # type: ignore

from sw_watchdog_msgs.msg import Heartbeat  # type: ignore


@pytest.mark.launch_test
def generate_test_description() -> LaunchDescription:  # type: ignore
    """Generate a launch configuration for this test."""
    demoLaunchFile = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('sw_watchdog_test'), 'launch'
                ),
                '/watchdog_test.launch.py',
            ]
        )
    )

    return LaunchDescription(
        [
            demoLaunchFile,
            # Signal that the test setup is complete
            ReadyToTest(),
        ]
    )


class WatchdogIntegTest(unittest.TestCase):
    """Watchdog integration test class."""

    def test_watchdog_integ(self) -> None:
        """Watchdog integration test case."""
        testExecutable = os.path.join(
            get_package_prefix('sw_watchdog_test'),
            'bin',
            'watchdog_integ_test',
        )
        topic_list = [
            ('/node_a/heartbeat', Heartbeat),
            ('/node_b/heartbeat', Heartbeat),
        ]
        with WaitForTopics(topic_list, timeout=15.0):
            result = subprocess.run(testExecutable)  # nosec
            assert 0 == result.returncode  # nosec
