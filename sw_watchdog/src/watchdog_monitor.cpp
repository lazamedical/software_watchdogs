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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sw_watchdog_msgs/msg/status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "sw_watchdog/watchdog_monitor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace sw_watchdog
{
WatchdogMonitor::WatchdogMonitor(rclcpp::NodeOptions options)
: Node("watchdog_monitor", options.start_parameter_event_publisher(false).
    start_parameter_services(false))
{
  declare_parameter("nodes_list", rclcpp::PARAMETER_STRING_ARRAY);

  // If it is not possible to get nodes list as parameter, clear it to be sure there is no leftover
  if (!get_parameter("nodes_list", nodes_list_)) {
    RCLCPP_ERROR(get_logger(), "The nodes_list parameter is missing!");
    nodes_list_.clear();
  }

  // Fill in the lists of states and create subscriber for each node in the nodes_list parameter
  for (const std::string & node_name : nodes_list_) {
    node_statuses_[node_name] = false;

    heartbeat_subs_[node_name] = create_subscription<sw_watchdog_msgs::msg::Status>(
      node_name + node_status_topic_postfix_,
      1,
      std::bind(&WatchdogMonitor::status_callback, this, _1));
  }

  // Create the publisher for Status aka Green Light. Keep the last
  // message so that new subscribers will get the current status value.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  status_pub_ = create_publisher<std_msgs::msg::Bool>(status_topic_name_, qos);

  // Create get_status service for getting the most current information about the state
  get_status_server_ = this->create_service<std_srvs::srv::Trigger>(
    get_status_serv_name_,
    std::bind(&WatchdogMonitor::get_status_handler, this, _1, _2, _3));

  // Create bringup service for initialisation of the bringup sequence
  bringup_server_ = this->create_service<std_srvs::srv::Trigger>(
    bringup_serv_name_,
    std::bind(&WatchdogMonitor::bringup_handler, this, _1, _2, _3));
}

/**
 * @brief Get the current overall status of all Watchdogs
 *
 * @return true if all Watchdogs report the nodes are running
 * @return false if at least 1 Watchdog reports the node is not running
 */
bool WatchdogMonitor::get_current_status() const
{
  bool result = true;

  // In case the list of nodes is empty, return false, otherwise iterate through it
  // and get cumulative status of all Watchdogs
  if (nodes_list_.empty()) {
    result = false;
  } else {
    for (const std::string & node_name : nodes_list_) {
      if (node_statuses_.find(node_name) != node_statuses_.end()) {
        result &= node_statuses_.at(node_name);
      }
    }
  }

  return result;
}

/**
 * @brief Callback after receiving the status topic from Watchdog
 *
 */
void WatchdogMonitor::status_callback(const sw_watchdog_msgs::msg::Status & msg)
{
  bool result = false;

  RCLCPP_INFO(
    get_logger(), "Received status: %s is %s",
    msg.sender.c_str(),
    msg.missed_number == 0 ? "alive" : "not alive");

  // In case there is a value for reported node, set it to the received status
  if (node_statuses_.find(msg.sender) != node_statuses_.end()) {
    node_statuses_[msg.sender] = (msg.missed_number == 0);
  } else {
    RCLCPP_ERROR(get_logger(), "Status of node \"%s\" was not found!", msg.sender.c_str());
  }

  // Get the current system state
  result = get_current_status();

  RCLCPP_INFO(get_logger(), "Sending out the status: %s", result ? "true" : "false");

  // Report current System state what might be changed after receiving this status message
  std_msgs::msg::Bool status_msg;
  status_msg.data = result;
  status_pub_->publish(status_msg);
}

/**
 * @brief Handler for the "get_status" service call
 *
 */
void WatchdogMonitor::get_status_handler(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  (void)request;
  RCLCPP_INFO(get_logger(), "Handling get_status request.");

  // Respond with the current System state
  response->success = get_current_status();
}

/**
 * @brief Handler for the "bringup" service call
 *
 */
void WatchdogMonitor::bringup_handler(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  (void)request;
  RCLCPP_INFO(get_logger(), "Handling bringup sequence (return True).");

  // Currently there is nothing to handle, so just return "true" value
  response->success = true;
}

}  // namespace sw_watchdog

RCLCPP_COMPONENTS_REGISTER_NODE(sw_watchdog::WatchdogMonitor)
