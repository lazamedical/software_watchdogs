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

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sw_watchdog/windowed_watchdog.hpp"

#include "gtest/gtest.h"

rclcpp::NodeOptions options;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("generic_node") {}

  void sendHeartbeat(sw_watchdog_msgs::msg::Heartbeat messageContent)
  {
    if (!HeartbeatPublisher) {
      HeartbeatPublisher =
        this->create_publisher<sw_watchdog_msgs::msg::Heartbeat>("heartbeat", 1);
    }

    HeartbeatPublisher->publish(messageContent);
  }

private:
  rclcpp::Publisher<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr HeartbeatPublisher;
};

class WindowedWatchdogTest : public testing::TestWithParam<bool>
{
protected:
  WindowedWatchdogTest()
  {
    options = rclcpp::NodeOptions();
  }

  ~WindowedWatchdogTest() override
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    std::vector<std::string> args{"dummy", "100", "3", "--publish", "--activate"};

    // If the test param is true, then add --keep-active
    if (GetParam()) {
      args.push_back("--keep-active");
    }

    options.arguments(args);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_P(WindowedWatchdogTest, ErrorTest)
{
  options.arguments({});
  (void)(::testing::GTEST_FLAG(death_test_style) = "threadsafe");

  EXPECT_EXIT(
    auto node = std::make_shared<sw_watchdog::WindowedWatchdog>(options),
    testing::ExitedWithCode(0),
    ".*") << "The node did not exit as expected!";
}

TEST_P(WindowedWatchdogTest, ContructorTest)
{
  auto node = std::make_shared<sw_watchdog::WindowedWatchdog>(options);

  ASSERT_TRUE(node != nullptr) << "The Watchdog object was not created!";
}

TEST_P(WindowedWatchdogTest, PublishStatusTest)
{
  auto node = std::make_shared<sw_watchdog::WindowedWatchdog>(options);
  ASSERT_NO_THROW(node->publish_status(1)) << "Publish_status threw an exception!";
}

TEST_P(WindowedWatchdogTest, OnCleanupTest)
{
  auto node = std::make_shared<sw_watchdog::WindowedWatchdog>(options);
  sw_watchdog::NodeCallback retVal = node->on_cleanup(rclcpp_lifecycle::State());

  ASSERT_EQ(retVal, sw_watchdog::NodeCallback::SUCCESS) << "Cleanup failed!";
}

TEST_P(WindowedWatchdogTest, OnShutdownTest)
{
  auto node = std::make_shared<sw_watchdog::WindowedWatchdog>(options);
  sw_watchdog::NodeCallback retVal = node->on_shutdown(rclcpp_lifecycle::State());

  ASSERT_EQ(retVal, sw_watchdog::NodeCallback::SUCCESS) << "Shutdown failed!";
}

TEST_P(WindowedWatchdogTest, BeatingTest)
{
  auto node = std::make_shared<sw_watchdog::WindowedWatchdog>(options);
  EXPECT_EQ(
    rcutils_logging_set_logger_level(
      node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG),
    RCUTILS_RET_OK);
  auto tester = std::make_shared<TestPublisher>();
  sw_watchdog_msgs::msg::Heartbeat message;
  bool expectedStateReached = false;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(tester);

  message.header.stamp = tester->get_clock()->now();

  tester->sendHeartbeat(message);

  for (int i = 0; i < 20; ++i) {
    testing::internal::CaptureStderr();
    executor.spin_some();
    if (testing::internal::GetCapturedStderr()
      .find("Watchdog raised, heartbeat sent at") != std::string::npos)
    {
      expectedStateReached = true;
      break;
    }
  }

  ASSERT_TRUE(expectedStateReached) << "Heartbeat was not accepted by WindowedWatchdog!";

  executor.remove_node(tester);

  expectedStateReached = false;
  for (int i = 0; i < 50; ++i) {
    testing::internal::CaptureStderr();
    using namespace std::chrono_literals;
    executor.spin_once(100ms);
    if (testing::internal::GetCapturedStderr()
      .find("on_deactivate() is called.") != std::string::npos)
    {
      expectedStateReached = true;
      break;
    }
  }

  if (GetParam()) {
    ASSERT_FALSE(expectedStateReached) <<
      "Watchdog transitioned to deactivated state with --keep-active.";
  } else {
    ASSERT_TRUE(expectedStateReached) <<
      "Watchdog did not transition to deactivated state without --keep-active.";
  }
}

INSTANTIATE_TEST_SUITE_P(
  WindowedWatchdogTest_P, WindowedWatchdogTest,
  ::testing::Values(false, true));

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
