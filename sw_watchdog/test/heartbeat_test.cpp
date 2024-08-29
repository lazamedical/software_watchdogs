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
#include "sw_watchdog/simple_heartbeat.hpp"

#include "gtest/gtest.h"

rclcpp::NodeOptions options;

class HeartbeatTest : public testing::Test
{
protected:
  HeartbeatTest()
  {
    options = rclcpp::NodeOptions();
  }

  ~HeartbeatTest() override
  {
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(HeartbeatTest, HelpTest)
{
  options.arguments({"-h"});

  (void)(::testing::GTEST_FLAG(death_test_style) = "threadsafe");

  EXPECT_EXIT(
    auto node = std::make_shared<sw_watchdog::SimpleHeartbeat>(options),
    testing::ExitedWithCode(0),
    ".*") << "The node did not exit as expected!";
}

TEST_F(HeartbeatTest, CallbackTest)
{
  bool callbackCalled = false;

  options.append_parameter_override("period", 100);

  auto node = std::make_shared<sw_watchdog::SimpleHeartbeat>(options);

  auto sub = node->create_subscription<sw_watchdog_msgs::msg::Heartbeat>(
    "heartbeat", 10,
    [&](const sw_watchdog_msgs::msg::Heartbeat::SharedPtr) -> void {
      callbackCalled = true;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::string output;

  for (int i = 0; i < 50 && !callbackCalled; ++i) {
    executor.spin_once();
  }

  ASSERT_TRUE(callbackCalled) << "Callback was not called!";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
