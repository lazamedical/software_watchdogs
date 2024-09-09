# SW Watchdog

[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://github.com/micro-ROS/system_modes/blob/master/LICENSE)
[![Build status](https://github.com/ros-safety/software_watchdogs/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/ros-safety/software_watchdogs/actions)

A library of (software) watchdogs based on timer and ROS 2 [lifecycle nodes](https://github.com/ros2/demos/blob/master/lifecycle/README.rst). This implementation is ready to be used in [Zenoh RMW](https://github.com/ros2/rmw_zenoh) environment.

This package includes a heartbeat node that can be added easily to an existing process via ROS 2 [node composition](https://index.ros.org/doc/ros2/Tutorials/Composition/).

## Overview

This package includes two watchdog implementations (*"readers"*) and a heartbeat node (a *"writer"*). A watchdog expects a heartbeat signal at the specified frequency and otherwise declares the *writer* to have failed. A failure results in the watchdog's life cycle [state machine](https://design.ros2.org/articles/node_lifecycle.html) to transition to the `Inactive` state along with emitting the corresponding state transition event. A system-level response can be implemented in the event handler to realize patterns such as cold standby, process restarts, *etc.*. The transition to `Inactive` state can be avoided by setting the *"--keep-active"* option. Then the watchdog only publishes the updated status without transition to `Inactive` state.

Multiple heartbeats can monitor multiple nodes or layers of the system SW. Then multiple watchdogs are started to monitor corresponding heartbeats. In this case the watchdog monitor node can be used to monitor the health of whole system. Watchdog monitor publishes the status every time any watchdog state is changed (missed heartbeat or received first heartbeat). The service for getting the current status can be used as well to get the most up-to-date information about the whole system. Watchdog monitor needs to be configured with the list of monitored nodes (ROS2 parameter "nodes_list").

Diagnostics part of this package monitors the frequency of heartbeats using [ROS2 Diagnostics](https://github.com/ros/diagnostics/tree/ros2) and the system-level state. The frequency boundaries are calculated from desired frequency of heartbeat set in `period` ROS2 parameter.

* `SimpleWatchdog` declares the monitored topic as failed after a violation of the granted *lease* time. The implementation checks 10 times per lease time for received heartbeat.
* `WindowedWatchdog` declares the monitored topic as failed after the specified maximum number of *lease* violations. The implementation checks 10 times per lease time for received heartbeat.
* `SimpleHeartbeat` publishes a heartbeat signal at the specified frequency.
* `WatchdogMonitor` monitores the specified watchdogs. Once any of watchdog changes its state (first heartbeat or missed heartbeat) it publishes the state of whole system by topic. It can be prompted for the most current state of the system by service.

## Usage

The launch files included in this package demonstrate both node composition with a heartbeat signal and the configuration of a corresponding watchdog. Heartbeat_diagnostics package contains the launch file for diagnostics aggregator.

This implementation does not dependen on `rmw`.

Start the heartbeat and watchdog examples in separate terminals:
```
ros2 launch sw_watchdog heartbeat_composition.launch.py
ros2 launch sw_watchdog watchdog_lifecycle.launch.py

```
The first command composes a single process consisting of a ROS 2 `demo_nodes_cpp::Talker` with a `SimpleHeartbeat` set at *200ms*. The second command starts a `SimpleWatchdog` which grants a lease of *220ms* to the Heartbeat publisher. The watchdog will transition to the `Inactive` state as soon as the Heartbeat publisher violates the lease (*e.g.,* via CTRL+C in the first terminal). Since the watchdog is a lifecycle node, it can be re-activated to listen for a Heartbeat signal via:
```
ros2 lifecycle set simple_watchdog activate
```

To test the `WindowedWatchdog` replace the launch command in the second terminal with:
```
ros2 launch sw_watchdog windowed_watchdog_lifecycle.launch.py
```
This grants the Heartbeat publisher a maximum of three deadline misses. Deadline misses can be tested by inserting artificial delays in the publishing thread, for example.

To test the diagnostics start the aggregator in separate terminal (while running at least heartbeat):
```
ros2 launch heartbeat_diagnostics diagnostics.launch.py
```
This starts the diagnostics aggregator what will monitor the heartbeat frequency. The watchdog monitor will be always in "stale" state, because it is not running now. The tool `rqt` can be started to see the diagnostics messages. Open the plugin named `Diagnostics viewer`.

To test whole system with the watchdog monitor start this single launch file to bring up 2 nodes monitored by heartbeats, 2 corresponding watchdogs and the watchdog monitor to monitor them.
```
ros2 launch sw_watchdog_test watchdog_test.launch.py
```
To run the diagnostics start the aggregator in separate terminal:
```
ros2 launch heartbeat_diagnostics diagnostics.launch.py
```
This starts the diagnostics aggregator what will monitor the heartbeat frequency and the status of the whole system. 

## Requirements

This package includes custom messages.

To use the `heartbeat_composition.launch.py` example, the `ros-*-demo-nodes-cpp` must be installed.

## Compatibility

This code is built and tested under:

* [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html) with [Ubuntu 24.04](http://releases.ubuntu.com/24.04/).
* Default DDS rmw
* Zenoh rmw

# Copyright

Copyright (c) 2024 by Laza Medical.

All Rights Reserved.
