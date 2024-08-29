//
// Copyright (c) 2024 by Laza Medical.
// All Rights Reserved.
//

#ifndef TESTNODE_HPP_
#define TESTNODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/empty.hpp"
#include "sw_watchdog_msgs/msg/heartbeat.hpp"

namespace sw_watchdog_test
{
/**
 * \brief A test class example for monitored node
 */
class TestNode : public rclcpp::Node
{
public:
/**
 * \brief Construct a new Test Node object
 *
 * \param[in] options Node options
 */
  explicit TestNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr delay_sub_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr periodic_delay_sub_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr exit_sub_ = nullptr;
  rclcpp::Subscription<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr beating_sub_ = nullptr;

  int periodic_delay_value = 0;
};
}  // namespace sw_watchdog_test

#endif  // TESTNODE_HPP_
