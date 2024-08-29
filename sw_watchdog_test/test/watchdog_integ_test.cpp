//
// Copyright (c) 2024 by Laza Medical.
// All Rights Reserved.
//

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include "sw_watchdog_msgs/msg/status.hpp"
#include "sw_watchdog_msgs/msg/heartbeat.hpp"

#include "gtest/gtest.h"

class TestingNode : public rclcpp::Node
{
public:
  TestingNode()
  : Node("TestingNode")
  {
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    delay_pub = this->create_publisher<std_msgs::msg::Int16>(delay_topic, qos);
    exit_pub = this->create_publisher<std_msgs::msg::Empty>(exit_topic, qos);
    request = std::make_shared<std_srvs::srv::Trigger::Request>();
    get_status_client = this->create_client<std_srvs::srv::Trigger>(get_status_service);
    node_a_lifecycle_change_client_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>(node_a_lifecycle_service);
  }

  rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId GetClientStatus()
  {
    while (!get_status_client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(
        rclcpp::get_logger("WatchdogIntegTest"),
        "The service is not ready yet.");
    }

    return get_status_client->async_send_request(request);
  }

  const std::string delay_topic = "/node_a/delay";
  const std::string exit_topic = "/node_b/exit";
  const std::string beat_a_topic = "/node_a/heartbeat";
  const std::string beat_b_topic = "/node_b/heartbeat";
  const std::string watchdog_status_topic = "/my_namespace/watchdog_monitor/status";
  const std::string get_status_service = "/my_namespace/watchdog_monitor/get_status";
  const std::string node_a_lifecycle_service = "/node_a/watchdog/change_state";
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr delay_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr exit_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_status_client;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr node_a_lifecycle_change_client_;
  std_msgs::msg::Int16 delay_msg;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request;
};

class WatchdogIntegTest : public testing::Test
{
public:
  static void SetUpTestSuite()
  {
    // Initialize ROS2 node
    RCLCPP_INFO(rclcpp::get_logger("WatchdogIntegTest"), "Initializing ROS2 ...");
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    // Shutdown ROS2 node
    RCLCPP_INFO(rclcpp::get_logger("WatchdogIntegTest"), "Shutting down ROS2 node ...");
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<TestingNode>();

    // Wait for heartbeats and give the watchdog some time to start up
    auto message = sw_watchdog_msgs::msg::Heartbeat();
    rclcpp::wait_for_message(message, node, node->beat_a_topic, std::chrono::seconds(3));
    rclcpp::wait_for_message(message, node, node->beat_b_topic, std::chrono::seconds(3));
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    auto result = node->GetClientStatus();
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(
        node,
        result,
        std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS) << "Service call to get current status timed out";

    EXPECT_TRUE(result.get()->success) <<
      "The watchdog_monitor has a false status, when it should be true.";
  }

  void TearDown() override
  {
    node.reset();
  }

  void ActivateWatchdog()
  {
    auto activate_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    activate_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    while (!node->node_a_lifecycle_change_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(
        rclcpp::get_logger("WatchdogIntegTest"),
        "The node_a lifecycle service is not ready yet.");
    }

    auto future = node->node_a_lifecycle_change_client_->async_send_request(activate_request);

    EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node,
      future,
      std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS) << "Service call to set active status timed out";

    EXPECT_TRUE(future.get()->success) << "Failed to transition node_a to active state";
  }

  /// Delay publishing the heartbeat in milliseconds
  /// \param duration delay in milliseconds
  void DelayHeartbeat(int duration)
  {
    RCLCPP_INFO(rclcpp::get_logger("WatchdogIntegTest"), "Publish: DelayHeartbeat");

    node->delay_msg.data = duration;
    node->delay_pub->publish(node->delay_msg);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    bool received = node->delay_pub->wait_for_all_acked(std::chrono::milliseconds(1));

    // Spin until the message is received, or too many iterations
    // have passed.
    for (int i = 0; i < 10 && !received; ++i) {
      executor.spin_once(std::chrono::milliseconds(100));
      received = node->delay_pub->wait_for_all_acked(std::chrono::milliseconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("WatchdogIntegTest"),
        "The delay message was %sreceived.", received ? "" : "still NOT ");
    }
  }

  /// \brief Simulate a node crash
  void SimulateNodeCrash()
  {
    RCLCPP_INFO(rclcpp::get_logger("WatchdogIntegTest"), "Publish: NodeCrash");
    std_msgs::msg::Empty exit_msg;
    node->exit_pub->publish(exit_msg);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    bool received = node->exit_pub->wait_for_all_acked(std::chrono::milliseconds(1));

    // Spin until the message is received, or too many iterations
    // have passed.
    for (int i = 0; i < 10 && !received; ++i) {
      executor.spin_once(std::chrono::milliseconds(100));
      received = node->exit_pub->wait_for_all_acked(std::chrono::milliseconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("WatchdogIntegTest"),
        "The crash message was %sreceived.", received ? "" : "still NOT ");
    }
  }

  std::shared_ptr<TestingNode> node = nullptr;
};

TEST_F(WatchdogIntegTest, DelayTest)
{
  // Delay the heartbeat by 10ms
  DelayHeartbeat(10);

  // Wait for a heartbeat what will come after the delay was applied
  auto message = sw_watchdog_msgs::msg::Heartbeat();
  rclcpp::wait_for_message(message, node, node->beat_a_topic, std::chrono::seconds(3));

  auto result = node->GetClientStatus();
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node,
      result,
      std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS) << "Service call to get current status timed out";

  // The system must be running if node is delayed by 10ms
  EXPECT_TRUE(result.get()->success) << "The system is not running after 10ms delay";

  // Delay the heartbeat by 400ms
  DelayHeartbeat(400);

  // Wait for a heartbeat what will come after the delay was applied
  rclcpp::wait_for_message(message, node, node->beat_a_topic, std::chrono::seconds(3));

  result = node->GetClientStatus();
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node,
      result,
      std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS) << "Service call to get current status timed out";

  // The system must be stopped after 400ms delay
  EXPECT_FALSE(result.get()->success) << "The system still running after 400ms delay";

  // Need to reactive the watchdog
  ActivateWatchdog();

  // Wait for a heartbeat to be accepted by watchdog
  rclcpp::wait_for_message(message, node, node->beat_a_topic, std::chrono::seconds(3));
  rclcpp::sleep_for(std::chrono::milliseconds(10));

  result = node->GetClientStatus();
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node,
      result,
      std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS) << "Service call to get current status timed out";

  // The system must be running
  EXPECT_TRUE(result.get()->success) << "The system is not running after 0ms delay";
}

TEST_F(WatchdogIntegTest, CrashTest)
{
  SimulateNodeCrash();
  auto message = sw_watchdog_msgs::msg::Heartbeat();

  // This is expected to timeout since the heartbeat won't be published after the node crashes
  // It may take some time for node to crash so ignore few heartbeats if received.
  bool retVal = true;
  for (int i = 0; (i < 5) && retVal; ++i) {
    retVal = rclcpp::wait_for_message(message, node, node->beat_b_topic, std::chrono::seconds(3));
  }
  EXPECT_FALSE(retVal) << "The node is still beating after its crash";

  // The system must be stopped after node exit
  auto result = node->GetClientStatus();
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(
      node,
      result,
      std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS) << "Service call to get current status timed out";

  EXPECT_FALSE(result.get()->success) << "The system is not stopped after node crashed";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
