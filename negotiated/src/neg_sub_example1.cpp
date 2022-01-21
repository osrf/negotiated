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

#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_subscriber.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("neg_sub_node");

  auto user_cb = [node](const std_msgs::msg::String & msg)
    {
      RCLCPP_INFO(rclcpp::get_logger("neg_sub_example1"), "User callback: %s", msg.data.c_str());
    };

  negotiated::SupportedTypeMap supported_type_map;
  supported_type_map.add_to_map<std_msgs::msg::String>("std_msgs/msg/String", "a", 1.0);
  supported_type_map.add_to_map<int>("int", "b", 1.0);

  auto neg_sub = std::make_shared<negotiated::NegotiatedSubscriber<std_msgs::msg::String>>(
    node,
    supported_type_map,
    "myneg",
    user_cb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
