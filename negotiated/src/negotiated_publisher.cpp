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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "std_msgs/msg/empty.hpp"

#include "negotiated/negotiated_publisher.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(rclcpp::Node::SharedPtr node, const std::string & topic_name)
  : topic_name_(topic_name),
    node_(node)
{
  publisher_ = rclcpp::create_publisher<std_msgs::msg::Empty>(node, topic_name, rclcpp::QoS(10));
}

void NegotiatedPublisher::negotiate()
{
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph = node_->get_node_graph_interface();
  std::vector<rclcpp::TopicEndpointInfo> sub_info_list = node_graph->get_subscriptions_info_by_topic(topic_name_);

  for (const rclcpp::TopicEndpointInfo & info : sub_info_list) {
    std::string name = info.node_name();
    std::string ns = info.node_namespace();

    fprintf(stderr, "Attempting to negotiate with %s/%s\n", name.c_str(), ns.c_str());

    //rclcpp::Client<negotiated_interfaces::srv::NegotiatedPreferences>::SharedPtr client =
  }
}

}  // namespace negotiated
