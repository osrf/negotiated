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

#ifndef NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
#define NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "std_msgs/msg/empty.hpp"

namespace negotiated
{

class NegotiatedPublisher
{
public:
  explicit NegotiatedPublisher(rclcpp::Node::SharedPtr node, const std::string & topic_name);

  bool negotiate();

private:
  std::string topic_name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr neg_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
