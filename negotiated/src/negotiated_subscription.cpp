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

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_subscription.hpp"

namespace negotiated
{

NegotiatedSubscription::NegotiatedSubscription(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name)
: node_(node),
  topic_name_(topic_name)
{
  auto sub_cb =
    [this, node](const negotiated_interfaces::msg::NewTopicInfo & msg)
    {
      // Only recreate the subscription if it is different than before
      if (msg.ros_type_name != ros_type_name_ || msg.format_match != format_match_) {
        ros_type_name_ = msg.ros_type_name;
        format_match_ = msg.format_match;
        auto factory = supported_type_map_.get_sub_factory(ros_type_name_, format_match_);
        this->subscription_ = factory(msg.topic_name);
      }
    };

  neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
    topic_name, rclcpp::QoS(10), sub_cb);
}

void NegotiatedSubscription::start()
{
  supported_types_pub_ = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    topic_name_ + "/supported_types",
    rclcpp::QoS(100).transient_local());

  supported_types_pub_->publish(supported_type_map_.get_types());
}

}  // namespace negotiated
