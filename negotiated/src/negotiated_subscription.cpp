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

#include "negotiated_interfaces/msg/negotiated_topic_info.hpp"
#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_subscription.hpp"

namespace negotiated
{

NegotiatedSubscription::NegotiatedSubscription(
  rclcpp::Node * node,
  const std::string & topic_name,
  const NegotiatedSubscriptionOptions & options)
: node_parameters_(node->get_node_parameters_interface()),
  node_topics_(node->get_node_topics_interface())
{
  auto sub_cb =
    [this, options](const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)
    {
      if (!msg.success && options.disconnect_on_negotiation_failure) {
        // We know the publisher attempted to and failed negotiation amongst the
        // various subscriptions.  We also know that it is no longer publishing
        // anything, so disconnect ourselves and hope for a better result next time.
        subscription_.reset();
        return;
      }

      negotiated_interfaces::msg::NegotiatedTopicInfo matched_info;
      std::string key;

      for (const negotiated_interfaces::msg::NegotiatedTopicInfo & info : msg.negotiated_topics) {
        if (info.ros_type_name == ros_type_name_ &&
          info.supported_type_name == supported_type_name_)
        {
          // The publisher renegotiated, but still supports the one we were already
          // connected to.  No more work to be done here.
          return;
        }

        std::string tmp_key = generate_key(info.ros_type_name, info.supported_type_name);
        if (key_to_supported_types_.count(tmp_key) == 0) {
          // This is not a combination we support, so we can't subscribe
          continue;
        }

        // Otherwise, this is one that we can support, so just choose it
        matched_info = info;
        key = tmp_key;
        break;
      }

      if (key.empty()) {
        // The publisher didn't give us anything that we can successfully subscribe to,
        // so just don't try.
        return;
      }

      ros_type_name_ = matched_info.ros_type_name;
      supported_type_name_ = matched_info.supported_type_name;

      auto sub_factory = key_to_supported_types_[key].sub_factory;
      subscription_ = sub_factory(matched_info.topic_name);
    };

  neg_subscription_ = rclcpp::create_subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    node_parameters_,
    node_topics_,
    topic_name,
    rclcpp::QoS(10),
    sub_cb);

  supported_types_pub_ = rclcpp::create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    node_parameters_,
    node_topics_,
    topic_name + "/supported_types",
    rclcpp::QoS(100).transient_local());
}

std::string NegotiatedSubscription::generate_key(
  const std::string & ros_type_name,
  const std::string & supported_type_name)
{
  return ros_type_name + "+" + supported_type_name;
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
