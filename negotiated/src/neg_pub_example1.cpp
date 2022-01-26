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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("neg_pub_node");

  negotiated::SupportedTypeMap supported_type_map;
  supported_type_map.add_supported_info<std_msgs::msg::String>(
    "std_msgs/msg/String", "a", 1.0);
  supported_type_map.add_supported_info<std_msgs::msg::Int32>(
    "std_msgs/msg/Int32", "a", 0.5);

  auto neg_pub = std::make_shared<negotiated::NegotiatedPublisher>(
    node,
    supported_type_map,
    "myneg");

  int count = 0;
  auto publish_message = [&count, &neg_pub]() -> void
    {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello World: " + std::to_string(count++);
      neg_pub->publish(msg);
    };

  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    std::chrono::seconds(1),
    publish_message);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
