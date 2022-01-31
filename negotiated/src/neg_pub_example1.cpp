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

#include "negotiated/negotiated_publisher.hpp"

#include "example_type_info.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("neg_pub_node");

  auto neg_pub = std::make_shared<negotiated::NegotiatedPublisher>(
    node,
    "myneg");
  neg_pub->add_supported_info<negotiated_examples::StringT>(1.0, rclcpp::QoS(1));
  neg_pub->add_supported_info<negotiated_examples::Int32T>(0.5, rclcpp::QoS(1));
  neg_pub->start();

  int count = 0;
  auto publish_message = [&count, &neg_pub]() -> void
    {
      if (neg_pub->type_was_negotiated<negotiated_examples::StringT>()) {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello World: " + std::to_string(count++);
        neg_pub->publish(msg);
      }

      if (neg_pub->type_was_negotiated<negotiated_examples::Int32T>()) {
        auto msg = std_msgs::msg::Int32();
        msg.data = count++;
        neg_pub->publish(msg);
      }
    };

  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    std::chrono::seconds(1),
    publish_message);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
