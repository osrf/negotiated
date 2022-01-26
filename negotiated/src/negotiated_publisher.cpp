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
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_publisher.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(
  rclcpp::Node::SharedPtr node,
  const SupportedTypeMap & supported_type_map,
  const std::string & topic_name,
  const rclcpp::QoS final_qos)
: node_(node),
  supported_type_map_(supported_type_map),
  topic_name_(topic_name),
  final_qos_(final_qos)
{
  neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NewTopicInfo>(
    topic_name_, rclcpp::QoS(10));

  auto neg_cb = [this](const negotiated_interfaces::msg::SupportedTypes & supported_types)
    {
      for (const negotiated_interfaces::msg::SupportedType & type :
        supported_types.supported_types)
      {
        RCLCPP_INFO(node_->get_logger(), "Adding supported_types %s -> %f", type.name.c_str(), type.weight);
        this->sub_supported_types_.supported_types.push_back(type);
      }

      negotiate();
    };

  std::string supported_type_name = topic_name_ + "/supported_types";
  supported_types_sub_ = node_->create_subscription<negotiated_interfaces::msg::SupportedTypes>(
    supported_type_name, rclcpp::QoS(100).transient_local(), neg_cb);
}

void NegotiatedPublisher::negotiate()
{
  RCLCPP_INFO(node_->get_logger(), "Negotiating");

  if (sub_supported_types_.supported_types.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Skipping negotiation because of empty subscription supported types");
    return;
  }

  if (supported_type_map_.get_types().supported_types.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Skipping negotiation because of empty publisher supported types");
    return;
  }

  for (const negotiated_interfaces::msg::SupportedType & sub_type : sub_supported_types_.supported_types) {
    RCLCPP_INFO(node_->get_logger(), "Saw sub supported type %s -> %f", sub_type.name.c_str(), sub_type.weight);

    for (const negotiated_interfaces::msg::SupportedType & pub_type : supported_type_map_.get_types().supported_types) {
      RCLCPP_INFO(node_->get_logger(), "  Saw pub supported type %s -> %f", pub_type.name.c_str(), pub_type.weight);
    }
  }

  // TODO(clalancette): run the algorithm to choose the type

  // TODO(clalancette): What happens if the subscription supported_types are empty?
  // TODO(clalancette): What happens if the publisher supported_types are empty?

  // Now that we've run the algorithm and figured out what our actual publication
  // "type" is going to be, create the publisher and inform the subscriptions
  // the name of it.

  std::string new_topic_name = topic_name_ + "/yuv422";
  // TODO(clalancette): Make this the chosen type
  ros_type_name_ = "std_msgs/msg/String";

  auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();
  msg->ros_type_name = ros_type_name_;
  msg->topic_name = new_topic_name;

  publisher_ = node_->create_generic_publisher(new_topic_name, ros_type_name_, final_qos_);

  neg_publisher_->publish(std::move(msg));

  return;
}

}  // namespace negotiated
