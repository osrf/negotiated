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

  negotiated_interfaces::msg::SupportedTypes supported_types;

  negotiated_interfaces::msg::SupportedType supported_type_a;
  supported_type_a.name = "a";
  supported_type_a.weight = 1.0;
  supported_types.supported_types.push_back(supported_type_a);

  negotiated_interfaces::msg::SupportedType supported_type_b;
  supported_type_b.name = "b";
  supported_type_b.weight = 1.0;
  supported_types.supported_types.push_back(supported_type_b);

  negotiated_interfaces::msg::SupportedType supported_type_c;
  supported_type_c.name = "c";
  supported_type_c.weight = 1.0;
  supported_types.supported_types.push_back(supported_type_c);

  negotiated::SupportedTypeMap supported_type_map;
  supported_type_map.add_to_map(supported_types);

  auto neg_sub = std::make_shared<negotiated::NegotiatedSubscriber<std_msgs::msg::String>>(
    node,
    supported_type_map,
    "myneg",
    user_cb);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
