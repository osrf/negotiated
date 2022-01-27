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
  const std::string & topic_name,
  const rclcpp::QoS final_qos)
: node_(node),
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
        RCLCPP_INFO(
          node_->get_logger(), "Adding supported_types %s -> %f",
          type.name.c_str(), type.weight);
        subscription_names_to_weights_[type.ros_type_name].push_back(type.weight);

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

  if (subscription_names_to_weights_.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Skipping negotiation because of empty subscription supported types");
    return;
  }

  if (supported_type_map_.get_types().supported_types.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Skipping negotiation because of empty publisher supported types");
    return;
  }


  auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();

  // TODO(clalancette): This is a stupid negotiation algorithm that just computes the
  // highest weight.  Needs work.
  double max_weight = 0.0;
  for (const negotiated_interfaces::msg::SupportedType & pub_type :
    supported_type_map_.get_types().supported_types)
  {
    RCLCPP_INFO(
      node_->get_logger(), "  Saw pub supported type %s -> %f",
      pub_type.ros_type_name.c_str(), pub_type.weight);

    std::unordered_map<std::string,
      std::vector<double>>::iterator it = subscription_names_to_weights_.find(
      pub_type.ros_type_name);
    if (it != subscription_names_to_weights_.end()) {
      double current_weight = 0.0;
      for (double w : it->second) {
        current_weight += w;
      }
      current_weight += pub_type.weight;

      if (current_weight > max_weight) {
        max_weight = current_weight;
        RCLCPP_INFO(node_->get_logger(), "  Chose type %s", pub_type.ros_type_name.c_str());
        // TODO(clalancette): What if the sub names don't match the pub name?
        msg->topic_name = topic_name_ + "/" + pub_type.name;
        ros_type_name_ = pub_type.ros_type_name;
        msg->ros_type_name = ros_type_name_;
      }
    }
  }

  // Now that we've run the algorithm and figured out what our actual publication
  // "type" is going to be, create the publisher and inform the subscriptions
  // the name of it.

  publisher_ = node_->create_generic_publisher(msg->topic_name, msg->ros_type_name, final_qos_);

  neg_publisher_->publish(std::move(msg));

  return;
}

}  // namespace negotiated
