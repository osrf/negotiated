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

#include <functional>
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
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  const std::string & topic_name,
  const NegotiatedSubscriptionOptions & negotiated_sub_options)
: node_parameters_(node_parameters),
  node_topics_(node_topics),
  negotiated_sub_options_(negotiated_sub_options)
{
  negotiated_subscription_ =
    rclcpp::create_subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    node_parameters_,
    node_topics_,
    topic_name,
    rclcpp::QoS(10),
    std::bind(&NegotiatedSubscription::topicsInfoCb, this, std::placeholders::_1));

  supported_types_pub_ = rclcpp::create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    node_parameters_,
    node_topics_,
    topic_name + "/supported_types",
    rclcpp::QoS(100).transient_local());
}

void NegotiatedSubscription::topicsInfoCb(
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)
{
  if (!msg.success && negotiated_sub_options_.disconnect_on_negotiation_failure) {
    // We know the publisher attempted to and failed negotiation amongst the various subscriptions.
    // We also know that it is no longer publishing anything, so disconnect ourselves and hope for
    // a better result next time.
    ros_type_name_ = "";
    supported_type_name_ = "";
    subscription_.reset();
    return;
  }

  negotiated_interfaces::msg::NegotiatedTopicInfo matched_info;
  std::string key;

  for (const negotiated_interfaces::msg::NegotiatedTopicInfo & info : msg.negotiated_topics) {
    if (info.ros_type_name == ros_type_name_ && info.supported_type_name == supported_type_name_ &&
      negotiated_sub_options_.keep_existing_match_if_possible)
    {
      // The publisher renegotiated, but still supports the one we were already connected to.  We
      // keep using the old type to ensure we don't lose data.
      return;
    }

    std::string tmp_key = generate_key(info.ros_type_name, info.supported_type_name);
    if (key_to_supported_types_.count(tmp_key) == 0) {
      // This is not a combination we support, so we can't subscribe to it.
      continue;
    }

    // Otherwise, this is a supported type that the subscription knows about.  We choose the first
    // one in the list, in the assumption that the publisher gave the list to us in priority order.
    // Note that if we are already connected, we check the rest of the list to see if we are
    // already subscribed to a later entry.

    if (key.empty()) {
      matched_info = info;
      key = tmp_key;
    }

    if ((ros_type_name_.empty() && supported_type_name_.empty()) ||
      !negotiated_sub_options_.keep_existing_match_if_possible)
    {
      // Small optimization; if we've never connected before, we can stop searching as soon as we
      // find the first valid match.
      break;
    }
  }

  if (key.empty()) {
    // The publisher didn't give us anything that we can successfully subscribe to.
    ros_type_name_ = "";
    supported_type_name_ = "";
    subscription_.reset();
    return;
  }

  ros_type_name_ = matched_info.ros_type_name;
  supported_type_name_ = matched_info.supported_type_name;

  auto sub_factory = key_to_supported_types_[key].sub_factory;
  subscription_ = sub_factory(matched_info.topic_name);
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

size_t NegotiatedSubscription::get_negotiated_topic_publisher_count() const
{
  return negotiated_subscription_->get_publisher_count();
}

size_t NegotiatedSubscription::get_data_topic_publisher_count() const
{
  if (ros_type_name_.empty() && supported_type_name_.empty()) {
    return 0;
  }

  return subscription_->get_publisher_count();
}

}  // namespace negotiated
