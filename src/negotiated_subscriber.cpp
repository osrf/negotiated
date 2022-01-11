// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "negotiated/negotiated_subscriber.hpp"

namespace negotiated
{

NegotiatedSubscriber::NegotiatedSubscriber(rclcpp::Node & node, const std::string & topic_name)
{
  auto cb = [](const std_msgs::msg::Empty & msg)
  {
    (void)msg;
  };

  subscription_ = rclcpp::create_subscription<std_msgs::msg::Empty>(node, topic_name, rclcpp::QoS(10), cb, rclcpp::SubscriptionOptions());
}

}  // namespace negotiated
