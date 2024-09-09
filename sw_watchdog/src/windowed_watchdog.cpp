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
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rclcpp_components/register_node_macro.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcutils/logging_macros.h"

#include "sw_watchdog_msgs/msg/heartbeat.hpp"
#include "sw_watchdog_msgs/msg/status.hpp"
#include "sw_watchdog/visibility_control.h"
#include "sw_watchdog/windowed_watchdog.hpp"

constexpr char OPTION_AUTO_START[] = "--activate";
constexpr char OPTION_PUB_STATUS[] = "--publish";
constexpr char OPTION_KEEP_ACTIVE[] = "--keep-active";
constexpr char DEFAULT_TOPIC_NAME[] = "heartbeat";

namespace
{

void print_usage()
{
  std::cout <<
    "Usage: windowed_watchdog lease max-misses [" << OPTION_AUTO_START << "] [-h]\n\n"
    "required arguments:\n"
    "\tlease: Lease in positive integer milliseconds granted to the watched entity.\n"
    "\tmax-misses: The maximum number of lease violations granted to the watched entity.\n"
    "optional arguments:\n"
    "\t" << OPTION_AUTO_START << ": Start the watchdog on creation.  Defaults to false.\n"
    "\t" << OPTION_PUB_STATUS << ": Publish lease expiration of the watched entity.  "
    "Defaults to false.\n"
    "\t" << OPTION_KEEP_ACTIVE << ": Keep the watchdog active, so that it automatically "
    "resubscribes to heartbeats. Default to false.\n"
    "\t-h : Print this help message." <<
    std::endl;
}

}  // namespace

namespace sw_watchdog
{

SW_WATCHDOG_PUBLIC
WindowedWatchdog::WindowedWatchdog(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("windowed_watchdog", options),
  autostart_(false), enable_pub_(false), topic_name_(DEFAULT_TOPIC_NAME),
  lease_misses_(0)
{
  // Parse node arguments
  const std::vector<std::string> & args = this->get_node_options().arguments();
  std::vector<char *> cargs;
  cargs.reserve(args.size());
  for (size_t i = 0; i < args.size(); ++i) {
    cargs.push_back(const_cast<char *>(args[i].c_str()));
  }

  if (args.size() < 3 || rcutils_cli_option_exist(&cargs[0], &cargs[0] + cargs.size(), "-h")) {
    print_usage();
    // TODO(anybody): Update the rclcpp_components template to be able to handle
    // exceptions. Raise one here, so stack unwinding happens gracefully.
    std::exit(0);
  }

  // Make sure the lease is more than zero.
  auto provided_lease = std::max(1ul, std::stoul(args[1]));

  // Lease duration must be >= heartbeat's lease duration
  lease_duration_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::milliseconds(provided_lease));
  max_misses_ = std::stoul(args[2]);

  if (rcutils_cli_option_exist(&cargs[0], &cargs[0] + cargs.size(), OPTION_AUTO_START)) {
    autostart_ = true;
  }
  if (rcutils_cli_option_exist(&cargs[0], &cargs[0] + cargs.size(), OPTION_PUB_STATUS)) {
    enable_pub_ = true;
  }
  if (rcutils_cli_option_exist(&cargs[0], &cargs[0] + cargs.size(), OPTION_KEEP_ACTIVE)) {
    keep_active_ = true;
  }

  if (autostart_) {
    configure();
    activate();
  }
}

void WindowedWatchdog::publish_status(uint16_t misses)
{
  auto msg = std::make_unique<sw_watchdog_msgs::msg::Status>();
  rclcpp::Time now = this->get_clock()->now();
  msg->stamp = now;
  msg->missed_number = misses;

  // Print the current state for demo purposes
  if (!status_pub_->is_activated()) {
    RCLCPP_INFO(get_logger(),
      "Lifecycle publisher is currently inactive. Messages are not published.");
  } else {
    RCLCPP_INFO(get_logger(),
      "Publishing lease expiry (missed count: %u) at [%f]",
      msg->missed_number, now.seconds());
  }

  // Only if the publisher is in an active state, the message transfer is
  // enabled and the message actually published.
  status_pub_->publish(std::move(msg));
}

NodeCallback WindowedWatchdog::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Initialize and configure node

  if (enable_pub_) {
    // Keep the last message so that new subscribers will get the
    // current status value.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    status_pub_ = create_publisher<sw_watchdog_msgs::msg::Status>("status", qos);
  }
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  return NodeCallback::SUCCESS;
}

NodeCallback WindowedWatchdog::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!heartbeat_sub_) {
    heartbeat_sub_ = create_subscription<sw_watchdog_msgs::msg::Heartbeat>(
      topic_name_,
      1,
      [this](const typename sw_watchdog_msgs::msg::Heartbeat::SharedPtr msg) -> void {
        if (last_heartbeat_.nanoseconds() == 0) {
          publish_status(0);
        }

        last_heartbeat_ = rclcpp::Time(msg->header.stamp);
        RCLCPP_DEBUG(
          get_logger(), "Watchdog raised, heartbeat sent at [%d.x]", msg->header.stamp.sec);
      });
  }

  // Reset existing timer and start it to check the heartbeat is received within lease_duration_
  if (timer_) {
    timer_.reset();
    timer_ = nullptr;
  }

  last_heartbeat_ = rclcpp::Time(0);

  // Check the heartbeat more often than lease duration to catch missed heartbeats.
  timer_ = this->create_timer(lease_duration_ / 10, [&](){
        if (last_heartbeat_.nanoseconds() == 0) {
          return;
        }
        int current_misses =
        (this->get_clock()->now() - last_heartbeat_).nanoseconds() /
        lease_duration_.count();

        bool ok = current_misses < max_misses_;

        if (lease_misses_ != current_misses) {
          publish_status(current_misses);
          lease_misses_ = current_misses;
        }

        if (!ok && !keep_active_) {
          deactivate();
        }
  });

  // Starting from this point, all messages are sent to the network.
  if (enable_pub_) {
    status_pub_->on_activate();
  }

  // Starting from this point, all messages are sent to the network.
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  return NodeCallback::SUCCESS;
}

NodeCallback WindowedWatchdog::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  heartbeat_sub_.reset();  // XXX there does not seem to be a 'deactivate' for subscribers.
  heartbeat_sub_ = nullptr;

  timer_.reset();
  timer_ = nullptr;

  // Starting from this point, all messages are no longer sent to the network.
  if (enable_pub_) {
    status_pub_->on_deactivate();
  }

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  return NodeCallback::SUCCESS;
}

NodeCallback WindowedWatchdog::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  timer_ = nullptr;

  status_pub_.reset();
  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");

  return NodeCallback::SUCCESS;
}

NodeCallback WindowedWatchdog::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  timer_.reset();
  timer_ = nullptr;

  heartbeat_sub_.reset();
  heartbeat_sub_ = nullptr;
  status_pub_.reset();

  RCUTILS_LOG_INFO_NAMED(
    get_name(),
    "on shutdown is called from state %s.",
    state.label().c_str()
  );

  return NodeCallback::SUCCESS;
}

}  // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(sw_watchdog::WindowedWatchdog)
