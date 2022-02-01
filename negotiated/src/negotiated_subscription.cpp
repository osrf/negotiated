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
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_subscription.hpp"

namespace negotiated
{

NegotiatedSubscription::NegotiatedSubscription(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name)
: node_(node)
{
  auto sub_cb =
    [this, node](const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)
    {
      // Only (re)create the subscription if it is different than before
      if (msg.ros_type_name == ros_type_name_ && msg.format_match == format_match_) {
        return;
      }

      std::string key = generate_key(msg.ros_type_name, msg.format_match);
      if (key_to_supported_types_.count(key) == 0) {
        // This is not a combination we support, so don't subscribe
        return;
      }

      ros_type_name_ = msg.ros_type_name;
      format_match_ = msg.format_match;

      auto sub_factory = key_to_supported_types_[key].sub_factory;
      this->subscription_ = sub_factory(msg.topic_name);
    };

  neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    topic_name, rclcpp::QoS(10), sub_cb);

  supported_types_pub_ = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    topic_name + "/supported_types",
    rclcpp::QoS(100).transient_local());
}

std::string NegotiatedSubscription::generate_key(
  const std::string & ros_type_name,
  const std::string & format_match)
{
  return ros_type_name + "+" + format_match;
}

void NegotiatedSubscription::start()
{
  auto supported_types = negotiated_interfaces::msg::SupportedTypes();
  for (const std::pair<const std::string, SupportedTypeInfo> & pair : key_to_supported_types_) {
    supported_types.supported_types.push_back(pair.second.supported_type);
  }

  supported_types_pub_->publish(supported_types);
}

}  // namespace negotiated
