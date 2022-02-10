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

namespace detail
{

negotiated_interfaces::msg::NegotiatedTopicInfo default_negotiate_cb(
  const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)
{
  negotiated_interfaces::msg::NegotiatedTopicInfo matched_info;

  if (!msg.success) {
    return matched_info;
  }

  for (const negotiated_interfaces::msg::NegotiatedTopicInfo & info : msg.negotiated_topics) {
    if (info.ros_type_name == existing_info.ros_type_name &&
      info.supported_type_name == existing_info.supported_type_name &&
      info.topic_name == existing_info.topic_name)
    {
      // The publisher renegotiated, but still supports the one we were already connected to.  We
      // keep using the old type to ensure we don't lose data.
      matched_info = info;
      break;
    }

    // Otherwise, this is a supported type that the subscription knows about.  We choose the first
    // one in the list, in the assumption that the publisher gave the list to us in priority order.
    // Note that if we are already connected, we check the rest of the list to see if we are
    // already subscribed to a later entry.

    if (matched_info.ros_type_name.empty() && matched_info.supported_type_name.empty() &&
      matched_info.topic_name.empty())
    {
      matched_info = info;
    }
  }

  return matched_info;
}

}  // namespace detail

NegotiatedSubscription::NegotiatedSubscription(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  const std::string & topic_name,
  const NegotiatedSubscriptionOptions & negotiated_sub_options)
: node_parameters_(node_parameters),
  node_topics_(node_topics),
  node_logging_(node_logging),
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
  negotiated_interfaces::msg::NegotiatedTopicsInfo supported_topics;
  supported_topics.success = msg.success;

  // Here we filter for only the topic types this subscription can support.
  for (const negotiated_interfaces::msg::NegotiatedTopicInfo & info : msg.negotiated_topics) {
    std::string tmp_key = generate_key(info.ros_type_name, info.supported_type_name);
    if (key_to_supported_types_.count(tmp_key) == 0) {
      // This is not a combination we support, so we can't subscribe to it.
      continue;
    }

    supported_topics.negotiated_topics.push_back(info);
  }

  negotiated_interfaces::msg::NegotiatedTopicInfo matched_info =
    negotiated_sub_options_.negotiate_cb(existing_topic_info_, supported_topics);

  if ((matched_info.ros_type_name.empty() || matched_info.supported_type_name.empty() ||
    matched_info.topic_name.empty()) && negotiated_sub_options_.disconnect_on_negotiation_failure)
  {
    // We know the publisher attempted to and failed negotiation amongst the various subscriptions.
    // We also know that it is no longer publishing anything, so disconnect ourselves and hope for
    // a better result next time.
    existing_topic_info_.ros_type_name = "";
    existing_topic_info_.supported_type_name = "";
    existing_topic_info_.topic_name = "";
    subscription_.reset();
    return;
  }

  if (matched_info.ros_type_name == existing_topic_info_.ros_type_name &&
    matched_info.supported_type_name == existing_topic_info_.supported_type_name &&
    matched_info.topic_name == existing_topic_info_.topic_name)
  {
    // This is exactly the same as what we are already connected to, so no work to do.
    return;
  }

  std::string key = generate_key(
    matched_info.ros_type_name,
    matched_info.supported_type_name);
  if (key_to_supported_types_.count(key) == 0) {
    RCLCPP_WARN(
      node_logging_->get_logger(),
      "Returned matched type from user callback is not supported, ignoring");
    return;
  }

  existing_topic_info_ = matched_info;

  subscription_ = key_to_supported_types_[key].sub_factory(existing_topic_info_.topic_name);
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
  if (existing_topic_info_.ros_type_name.empty() ||
    existing_topic_info_.supported_type_name.empty() || existing_topic_info_.topic_name.empty())
  {
    return 0;
  }

  return subscription_->get_publisher_count();
}

}  // namespace negotiated
