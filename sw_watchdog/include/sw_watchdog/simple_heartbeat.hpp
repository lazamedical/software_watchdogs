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

#ifndef SW_WATCHDOG__SIMPLE_HEARTBEAT_HPP_
#define SW_WATCHDOG__SIMPLE_HEARTBEAT_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sw_watchdog/visibility_control.h"
#include "sw_watchdog_msgs/msg/heartbeat.hpp"

namespace sw_watchdog
{
/**
 * A class that publishes heartbeats at a fixed frequency with the header set to current time.
 */
class SimpleHeartbeat : public rclcpp::Node
{
public:
  SW_WATCHDOG_PUBLIC
  //! \brief Node constructor.
  //!
  //! The constructor declares a "period" integer parameter in milliseconds.
  //! If the period is not specified, then a value of 200ms will be used.
  //! \param[in] options Node options
  explicit SimpleHeartbeat(rclcpp::NodeOptions options);

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sw_watchdog_msgs::msg::Heartbeat>::SharedPtr publisher_;
};
}  // namespace sw_watchdog

#endif  // SW_WATCHDOG__SIMPLE_HEARTBEAT_HPP_
