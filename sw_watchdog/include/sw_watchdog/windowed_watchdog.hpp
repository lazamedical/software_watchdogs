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

#ifndef SW_WATCHDOG__WINDOWED_WATCHDOG_HPP_
#define SW_WATCHDOG__WINDOWED_WATCHDOG_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/bool.hpp"
#include "sw_watchdog_msgs/msg/heartbeat.hpp"
#include "sw_watchdog_msgs/msg/status.hpp"
#include "sw_watchdog/visibility_control.h"

namespace sw_watchdog
{

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn NodeCallback;

/// WindowedWatchdog inheriting from rclcpp_lifecycle::LifecycleNode
/**
 * Internally relies on the QoS deadline and liveliness policies provided by the rmw implementation
 * (e.g., DDS). The lease passed to this watchdog has to be > the period of the heartbeat signal to
 * account for network transmission times.
 */
class WindowedWatchdog : public rclcpp_lifecycle::LifecycleNode
{
public:
  SW_WATCHDOG_PUBLIC
  explicit WindowedWatchdog(const rclcpp::NodeOptions &);

  /**
   * @brief Publish lease expiry of the watched entity
   */
  void publish_status(uint16_t misses);

  /**
   * @brief Transition callback for state configuring
   */
  NodeCallback on_configure(const rclcpp_lifecycle::State &);
  /**
   * @brief Transition callback for state activating
   */
  NodeCallback on_activate(const rclcpp_lifecycle::State &);
  /**
   * @brief Transition callback for state deactivating
   */
  NodeCallback on_deactivate(const rclcpp_lifecycle::State &);
  /**
   * @brief Transition callback for state cleaningup
   */
  NodeCallback on_cleanup(const rclcpp_lifecycle::State &);
  /**
   * @brief Transition callback for state shutting down
   */
  NodeCallback on_shutdown(const rclcpp_lifecycle::State & state);

private:
  typedef rclcpp_lifecycle::LifecyclePublisher<sw_watchdog_msgs::msg::Status> statusPubType;

  /// The lease duration granted to the remote (heartbeat) publisher
  std::chrono::milliseconds lease_duration_;
  rclcpp::Subscription<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr heartbeat_sub_ = nullptr;
  /// Publish lease expiry for the watched entity
  // By default, a lifecycle publisher is inactive by creation and has to be activated to publish.
  std::shared_ptr<statusPubType> status_pub_ = nullptr;
  /// Whether to enable the watchdog on startup. Otherwise, lifecycle transitions have to be raised
  bool autostart_;
  /// Whether a lease expiry should be published
  bool enable_pub_;
  /// Topic name for heartbeat signal by the watched entity
  const std::string topic_name_;
  /// The number of lease misses since the last heartbeat was received
  std::atomic<uint16_t> lease_misses_;
  /// The maximum number of lease misses granted to the watched entity
  uint16_t max_misses_;
  rclcpp::QoS qos_profile_;
  rclcpp::SubscriptionOptions heartbeat_sub_options_;
};
}  // namespace sw_watchdog

#endif  // SW_WATCHDOG__WINDOWED_WATCHDOG_HPP_
