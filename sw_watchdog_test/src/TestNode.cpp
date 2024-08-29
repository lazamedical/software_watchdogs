//
// Copyright (c) 2024 by Laza Medical.
// All Rights Reserved.
//

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sw_watchdog_msgs/msg/heartbeat.hpp"

#include "TestNode.hpp"

namespace sw_watchdog_test
{
TestNode::TestNode(rclcpp::NodeOptions options)
: Node("TestNode", options.start_parameter_event_publisher(false).
    start_parameter_services(false))
{
  RCLCPP_INFO(this->get_logger(), "Node %s has started.", this->get_fully_qualified_name());

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  // Periodic delay topic is used to simulate the busyness of the node in a periodic cycle
  // This feature can be used for testing the sensitivity of heartbeat periodicity/frequency.
  periodic_delay_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    "periodic_delay",
    qos,
    [this](const typename std_msgs::msg::Int16::SharedPtr msg) -> void {
      RCLCPP_INFO(get_logger(), "Going to delay the node periodically by: %d ms", msg->data);
      periodic_delay_value = msg->data;
    }
  );

  // Delay topic is used to simulate the busyness of the node for a longer time
  delay_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    "delay",
    qos,
    [this](const typename std_msgs::msg::Int16::SharedPtr msg) -> void {
      RCLCPP_INFO(get_logger(), "Delaying the node by: %d ms", msg->data);
      usleep(1000 * msg->data);
    }
  );

  // Exit topic is used to simulate the node crash
  exit_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "exit",
    qos,
    [this](const typename std_msgs::msg::Empty::SharedPtr msg) -> void {
      (void)msg;
      RCLCPP_INFO(get_logger(), "Exiting the node %s", this->get_fully_qualified_name());
      exit(-1);
    }
  );

  // Heartbeat topic is used as a timer for periodic delay.
  // This way the delay is always the same in regards to the Heartbeats.
  beating_sub_ = this->create_subscription<sw_watchdog_msgs::msg::Heartbeat>(
    std::string(this->get_namespace()) + "/heartbeat",
    1,
    [this](const typename sw_watchdog_msgs::msg::Heartbeat::SharedPtr msg) -> void {
      (void)msg;
      if (periodic_delay_value > 0) {
        RCLCPP_INFO(get_logger(), "Periodically delaying the node by: %d ms", periodic_delay_value);
        usleep(1000 * periodic_delay_value);
      }
    }
  );
}

}  // namespace sw_watchdog_test

RCLCPP_COMPONENTS_REGISTER_NODE(sw_watchdog_test::TestNode)
