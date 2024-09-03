//
// Copyright (c) 2024 by Laza Medical.
// All Rights Reserved.
//
// Copyright (c) 2020 Mapless AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp_components/register_node_macro.hpp"

#include "sw_watchdog_msgs/msg/heartbeat.hpp"
#include "sw_watchdog/visibility_control.h"
#include "sw_watchdog/simple_heartbeat.hpp"

// Buffer added to heartbeat to define lease.
constexpr std::chrono::milliseconds LEASE_DELTA = std::chrono::milliseconds(20);

namespace
{

void print_usage()
{
  std::cout <<
    "Usage: simple_heartbeat [-h] --ros-args -p period:=value [...]\n\n"
    "required arguments:\n"
    "\tperiod: Period in positive integer milliseconds of the heartbeat signal.\n"
    "optional arguments:\n"
    "\t-h : Print this help message." <<
    std::endl;
}

}  // namespace

namespace sw_watchdog
{

SW_WATCHDOG_PUBLIC
SimpleHeartbeat::SimpleHeartbeat(rclcpp::NodeOptions options)
: Node("simple_heartbeat", options.start_parameter_event_publisher(false).
    start_parameter_services(false))
{
  declare_parameter("period", rclcpp::PARAMETER_INTEGER);

  const std::vector<std::string> & args = this->get_node_options().arguments();
  // Parse node arguments
  if (std::find(args.begin(), args.end(), "-h") != args.end()) {
    print_usage();
    // TODO(anybody): Update the rclcpp_components template to be able to handle
    // exceptions. Raise one here, so stack unwinding happens gracefully.
    std::exit(0);
  }

  std::chrono::milliseconds heartbeat_period;
  try {
    heartbeat_period = std::chrono::milliseconds(get_parameter("period").as_int());
  } catch (...) {
    print_usage();
    // TODO(anybody): Update the rclcpp_components template to be able to handle
    // exceptions. Raise one here, so stack unwinding happens gracefully.
    std::exit(-1);
  }

  // The granted lease is essentially infite here, i.e., only reader/watchdog will notify
  // violations. XXX causes segfault for cyclone dds, hence pass explicit lease life > heartbeat.
  rclcpp::QoS qos_profile(1);
  qos_profile
  .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  .liveliness_lease_duration(heartbeat_period + LEASE_DELTA)
  .deadline(heartbeat_period + LEASE_DELTA);

  // assert liveliness on the 'heartbeat' topic with diagnostics
  updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  updater_->setHardwareID(this->get_fully_qualified_name());

  diagnostic_updater::FrequencyStatusParam freq_param(
    &diag_updater_min_freq_, &diag_updater_max_freq_, 0.01, 2);

  // assert liveliness on the 'heartbeat' topic
  publisher_ = this->create_publisher<sw_watchdog_msgs::msg::Heartbeat>(
    "heartbeat",
    qos_profile
  );

  diag_pub_ = std::make_unique<diag_pub_type>(
    publisher_, *updater_, freq_param,
    diagnostic_updater::TimeStampStatusParam());

  timer_ = this->create_wall_timer(heartbeat_period,
    std::bind(&SimpleHeartbeat::timer_callback, this));
}

void SimpleHeartbeat::timer_callback()
{
  auto message = sw_watchdog_msgs::msg::Heartbeat();
  rclcpp::Time now = this->get_clock()->now();
  message.header.stamp = now;
  RCLCPP_INFO(this->get_logger(), "Publishing heartbeat, sent at [%f]", now.seconds());
  diag_pub_->publish(message);
}

}  // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(sw_watchdog::SimpleHeartbeat)
