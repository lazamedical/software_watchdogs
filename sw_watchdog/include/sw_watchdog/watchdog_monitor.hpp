//
// Copyright (c) 2024 by Laza Medical.
// All Rights Reserved.
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

#ifndef SW_WATCHDOG__WATCHDOG_MONITOR_HPP_
#define SW_WATCHDOG__WATCHDOG_MONITOR_HPP_

#include <map>
#include <string>
#include <vector>
#include <memory>

#include "sw_watchdog_msgs/msg/status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"

namespace sw_watchdog
{
/**
 * A class that monitors the running watchdogs and publishes global statuses
 */
class WatchdogMonitor : public rclcpp::Node
{
public:
  explicit WatchdogMonitor(rclcpp::NodeOptions);

private:
  typedef rclcpp::Publisher<std_msgs::msg::Bool> statusPubType;
  typedef rclcpp::Subscription<sw_watchdog_msgs::msg::Status> statusSubType;

  /**
   * @brief Get the current overall status of all Watchdogs
   *
   * @return true if all Watchdogs report the nodes are running
   * @return false if at least 1 Watchdog reports the node is not running
   */
  bool get_current_status(void) const;

  /**
   * @brief Callback after receiving the status topic from Watchdog
   *
   */
  void status_callback(const sw_watchdog_msgs::msg::Status &);

  /**
   * @brief Handler for the "get_status" service call
   *
   */
  void get_status_handler(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>);

  /**
   * @brief Handler for the "bringup" service call
   *
   */
  void bringup_handler(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    const std::shared_ptr<std_srvs::srv::Trigger::Response>);

  // String constants used for the topic and service names
  const std::string node_status_topic_postfix_ = "/status";
  const std::string status_topic_name_ = "~/status";
  const std::string get_status_serv_name_ = "~/get_status";
  const std::string bringup_serv_name_ = "~/bringup";

  // Variables to store list of data required for each node
  std::map<std::string, bool> node_statuses_;
  std::map<std::string, statusSubType::SharedPtr> heartbeat_subs_;
  std::vector<std::string> nodes_list_;

  // Topic and service pointers
  std::shared_ptr<statusPubType> status_pub_ = nullptr;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_status_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bringup_server_;
};
}  // namespace sw_watchdog

#endif  // SW_WATCHDOG__WATCHDOG_MONITOR_HPP_
