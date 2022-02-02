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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_subscription.hpp"

#include "example_type_info.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("neg_sub_node");

  auto string_user_cb = [node](const std_msgs::msg::String & msg)
    {
      RCLCPP_INFO(node->get_logger(), "String user callback: %s", msg.data.c_str());
    };

  auto int_user_cb = [node](const std_msgs::msg::Int32 & msg)
    {
      RCLCPP_INFO(node->get_logger(), "Int user callback: %d", msg.data);
    };

  auto neg_sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node,
    "myneg");
  neg_sub->add_supported_callback<negotiated_examples::StringT>(
    1.0,
    rclcpp::QoS(1),
    string_user_cb);
  neg_sub->add_supported_callback<negotiated_examples::Int32T>(
    0.5,
    rclcpp::QoS(1),
    int_user_cb);
  neg_sub->start();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}