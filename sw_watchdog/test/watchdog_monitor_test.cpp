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

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sw_watchdog/watchdog_monitor.hpp"

#include "sw_watchdog_msgs/msg/status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "gtest/gtest.h"

rclcpp::NodeOptions options;

class WatchdogMonitorTest : public testing::Test
{
protected:
  WatchdogMonitorTest()
  {
    options = rclcpp::NodeOptions();
  }

  ~WatchdogMonitorTest() override
  {
  }

  void SetUp() override
  {
    std::vector<std::string> nodes_list;
    nodes_list.emplace_back("/node_a/watchdog");

    rclcpp::init(0, nullptr);
    options.append_parameter_override("nodes_list", nodes_list);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(WatchdogMonitorTest, ErrorTest)
{
  options.parameter_overrides().clear();

  testing::internal::CaptureStderr();
  auto node = std::make_shared<sw_watchdog::WatchdogMonitor>(options);

  ASSERT_NE(
    testing::internal::GetCapturedStderr().find("The nodes_list parameter is missing"),
    std::string::npos
  ) << "There should be the error because nodes_list is missing";
}

TEST_F(WatchdogMonitorTest, ContructorTest)
{
  auto node = std::make_shared<sw_watchdog::WatchdogMonitor>(options);

  ASSERT_NE(node, nullptr) << "The Watchdog object was not created!";
}

TEST_F(WatchdogMonitorTest, WatchdogStatusTest)
{
  auto node = std::make_shared<sw_watchdog::WatchdogMonitor>(options);
  auto tester = std::make_shared<rclcpp::Node>("generic_node");
  auto publisher = tester->create_publisher<sw_watchdog_msgs::msg::Status>(
    "node_a/watchdog/status",
    10
  );

  sw_watchdog_msgs::msg::Status message = sw_watchdog_msgs::msg::Status();
  bool expectedStateReached = false;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(tester);

  message.missed_number = 0;

  publisher->publish(message);

  for (int i = 0; i < 20; ++i) {
    testing::internal::CaptureStderr();
    executor.spin_some();
    if (testing::internal::GetCapturedStderr()
      .find("Received status:") != std::string::npos)
    {
      expectedStateReached = true;
      break;
    }
  }

  ASSERT_TRUE(expectedStateReached) << "Status reception was not handled correctly";
}

TEST_F(WatchdogMonitorTest, BringupTest)
{
  auto node = std::make_shared<sw_watchdog::WatchdogMonitor>(options);
  bool expectedStateReached = false;

  auto client = node->create_client<std_srvs::srv::Trigger>("/watchdog_monitor/bringup");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  client->async_send_request(request);

  for (int i = 0; i < 10; ++i) {
    testing::internal::CaptureStderr();
    executor.spin_some();
    if (testing::internal::GetCapturedStderr()
      .find("Handling bringup sequence") != std::string::npos)
    {
      expectedStateReached = true;
      break;
    }
  }

  ASSERT_TRUE(expectedStateReached) << "Bringup sequence was not handled correctly";
}

TEST_F(WatchdogMonitorTest, GetStatusTest)
{
  auto node = std::make_shared<sw_watchdog::WatchdogMonitor>(options);
  bool expectedStateReached = false;

  auto client = node->create_client<std_srvs::srv::Trigger>("/watchdog_monitor/get_status");
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  client->async_send_request(request);

  for (int i = 0; i < 10; ++i) {
    testing::internal::CaptureStderr();
    executor.spin_some();
    if (testing::internal::GetCapturedStderr()
      .find("Handling get_status request") != std::string::npos)
    {
      expectedStateReached = true;
      break;
    }
  }

  ASSERT_TRUE(expectedStateReached) << "Get status service was not handled correctly";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
