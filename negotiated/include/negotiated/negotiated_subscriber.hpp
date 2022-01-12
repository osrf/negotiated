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

#ifndef NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
#define NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/srv/negotiated_preferences.hpp"

namespace negotiated
{

class NegotiatedSubscriber
{
public:
  explicit NegotiatedSubscriber(rclcpp::Node::SharedPtr node, const std::string & topic_name);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  rclcpp::Service<negotiated_interfaces::srv::NegotiatedPreferences>::SharedPtr negotiation_srv_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
