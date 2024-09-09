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

// Default period if no one specified in parameter
constexpr std::chrono::milliseconds DEFAULT_PERIOD = std::chrono::milliseconds(200);

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

  std::chrono::milliseconds heartbeat_period = DEFAULT_PERIOD;
  try {
    heartbeat_period = std::chrono::milliseconds(get_parameter("period").as_int());

    // If the period is specified, compute the minimum and maximum frequencies for diagnostics
    // By default set the minimum frequency of heartbeat to desired one - 0.05 Hz
    diag_updater_min_freq_ = (1000 / heartbeat_period.count()) - 0.05;

    // By default set the maximum frequency of heartbeat to desired one + 0.5 Hz
    diag_updater_max_freq_ = (1000 / heartbeat_period.count()) + 0.5;
  } catch (...) {
    RCLCPP_WARN(this->get_logger(),
      "period is not specified, using default value of %ldms",
      DEFAULT_PERIOD.count());
  }

  // assert liveliness on the 'heartbeat' topic with diagnostics
  updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  updater_->setHardwareID(this->get_fully_qualified_name());

  diagnostic_updater::FrequencyStatusParam freq_param(
    &diag_updater_min_freq_, &diag_updater_max_freq_, 0.01, 2);

  publisher_ = this->create_publisher<sw_watchdog_msgs::msg::Heartbeat>("heartbeat", 1);
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
  RCLCPP_DEBUG(this->get_logger(), "Publishing heartbeat, sent at [%f]", now.seconds());
  diag_pub_->publish(message);
}

}  // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(sw_watchdog::SimpleHeartbeat)
