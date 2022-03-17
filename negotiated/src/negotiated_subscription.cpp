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
    std::bind(&NegotiatedSubscription::topics_info_cb, this, std::placeholders::_1));

  supported_types_pub_ = rclcpp::create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    node_parameters_,
    node_topics_,
    negotiated_subscription_->get_topic_name() + std::string("/_supported_types"),
    rclcpp::QoS(100).transient_local());
}

void NegotiatedSubscription::topics_info_cb(
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)
{
  negotiated_topics_.success = msg.success;
  negotiated_topics_.negotiated_topics.clear();

  // Here we filter for only the topic types this subscription can support.
  for (const negotiated_interfaces::msg::NegotiatedTopicInfo & info : msg.negotiated_topics) {
    std::string tmp_key = generate_key(info.ros_type_name, info.supported_type_name);
    if (key_to_supported_types_.count(tmp_key) == 0) {
      // This is not a combination we support, so we can't subscribe to it.
      continue;
    }

    if (downstream_key_to_supported_types_.size() > 0) {
      if (downstream_key_to_supported_types_.count(tmp_key) == 0) {
        continue;
      }
    }

    negotiated_topics_.negotiated_topics.push_back(info);
  }

  negotiated_interfaces::msg::NegotiatedTopicInfo matched_info =
    negotiated_sub_options_.negotiate_cb(existing_topic_info_, negotiated_topics_);

  if (matched_info.ros_type_name.empty() || matched_info.supported_type_name.empty() ||
    matched_info.topic_name.empty())
  {
    if (negotiated_sub_options_.disconnect_on_negotiation_failure) {
      // The negotiation failed for one reason or another, so disconnect ourselves and hope for
      // a better result on the next negotiation.
      existing_topic_info_.ros_type_name = "";
      existing_topic_info_.supported_type_name = "";
      existing_topic_info_.topic_name = "";
      subscription_.reset();
    }
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

  if (downstream_key_to_supported_types_.size() > 0) {
    if (downstream_key_to_supported_types_.count(key) == 0) {
      RCLCPP_WARN(
        node_logging_->get_logger(),
        "Returned matched type is not supported by downstreams, ignoring");
      return;
    }
  }

  existing_topic_info_ = matched_info;

  if (key_to_supported_types_[key].is_compat) {
    subscription_ = key_to_supported_types_[key].subscription;
    current_subscription_is_compat_ = true;
  } else {
    subscription_ = key_to_supported_types_[key].sub_factory(existing_topic_info_.topic_name);
    current_subscription_is_compat_ = false;
  }

  for (const std::shared_ptr<AfterSubscriptionCallbackHandle> & handle : after_subscription_cbs_) {
    handle->callback();
  }
}

std::string NegotiatedSubscription::generate_key(
  const std::string & ros_type_name,
  const std::string & supported_type_name)
{
  return ros_type_name + "+" + supported_type_name;
}

void NegotiatedSubscription::send_preferences()
{
  auto supported_types = negotiated_interfaces::msg::SupportedTypes();

  if (downstream_key_to_supported_types_.size() == 0) {
    for (const std::pair<const std::string, SupportedTypeInfo> & pair : key_to_supported_types_) {
      supported_types.supported_types.push_back(pair.second.supported_type);
    }
  } else {
    for (const std::pair<const std::string,
      std::vector<SupportedTypeInfo>> & pair : downstream_key_to_supported_types_)
    {
      // We only send along types that both this object and its downstreams support.
      if (key_to_supported_types_.count(pair.first) > 0) {
        supported_types.supported_types.push_back(pair.second[0].supported_type);
      }
    }
  }

  supported_types_pub_->publish(supported_types);
}

void NegotiatedSubscription::start()
{
  user_called_start_ = true;
  send_preferences();
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

const negotiated_interfaces::msg::NegotiatedTopicsInfo &
NegotiatedSubscription::get_negotiated_topics_info() const
{
  return negotiated_topics_;
}

std::shared_ptr<NegotiatedSubscription::AfterSubscriptionCallbackHandle>
NegotiatedSubscription::add_after_subscription_callback(
  const AfterSubscriptionCallbackFunction & cb)
{
  auto handle = std::make_shared<AfterSubscriptionCallbackHandle>();
  handle->callback = cb;
  after_subscription_cbs_.emplace_front(handle);
  return handle;
}

void NegotiatedSubscription::remove_after_subscription_callback(
  const NegotiatedSubscription::AfterSubscriptionCallbackHandle * const handle)
{
  auto it = std::find_if(
    after_subscription_cbs_.begin(),
    after_subscription_cbs_.end(),
    [handle](const std::shared_ptr<AfterSubscriptionCallbackHandle> & check_handle) {
      return handle == check_handle.get();
    });
  if (it != after_subscription_cbs_.end()) {
    after_subscription_cbs_.erase(it);
  } else {
    RCLCPP_WARN(node_logging_->get_logger(), "Attempted to remove callback that didn't exist");
  }
}

void NegotiatedSubscription::update_downstream_supported_types(
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_add,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_remove,
  const PublisherGid & gid)
{
  bool made_changes = false;

  for (const negotiated_interfaces::msg::SupportedType & type_to_remove :
    downstream_types_to_remove.supported_types)
  {
    std::string key_name = generate_key(
      type_to_remove.ros_type_name,
      type_to_remove.supported_type_name);
    if (downstream_key_to_supported_types_.count(key_name) == 0) {
      return;
    }

    auto it = std::find_if(
      downstream_key_to_supported_types_[key_name].begin(),
      downstream_key_to_supported_types_[key_name].end(),
      [&gid = std::as_const(gid)](const SupportedTypeInfo & type_info) {
        return type_info.gid == gid;
      });
    if (it != downstream_key_to_supported_types_[key_name].end()) {
      downstream_key_to_supported_types_[key_name].erase(it);
    }

    if (downstream_key_to_supported_types_[key_name].size() == 0) {
      downstream_key_to_supported_types_.erase(key_name);
      made_changes = true;
    }
  }

  for (const negotiated_interfaces::msg::SupportedType & type_to_add :
    downstream_types_to_add.supported_types)
  {
    std::string key_name = generate_key(type_to_add.ros_type_name, type_to_add.supported_type_name);

    if (downstream_key_to_supported_types_.count(key_name) == 0) {
      downstream_key_to_supported_types_.emplace(key_name, std::vector<SupportedTypeInfo>());
      made_changes = true;
    }

    bool gid_already_in_list{false};
    for (SupportedTypeInfo & type_info : downstream_key_to_supported_types_[key_name]) {
      if (type_info.gid == gid) {
        // The GID was already in the list, so just update things.
        type_info.supported_type.ros_type_name = type_to_add.ros_type_name;
        type_info.supported_type.supported_type_name = type_to_add.supported_type_name;
        type_info.supported_type.weight = type_to_add.weight;
        gid_already_in_list = true;
        break;
      }
    }

    if (!gid_already_in_list) {
      SupportedTypeInfo type_info;
      type_info.gid = gid;
      type_info.supported_type.ros_type_name = type_to_add.ros_type_name;
      type_info.supported_type.supported_type_name = type_to_add.supported_type_name;
      type_info.supported_type.weight = type_to_add.weight;

      downstream_key_to_supported_types_[key_name].push_back(type_info);
    }
  }

  // If the user has started negotiation we should resend our preferences
  if (user_called_start_ && made_changes) {
    send_preferences();
  }
}

void NegotiatedSubscription::remove_all_downstream_supported_types(
  const negotiated_interfaces::msg::SupportedTypes & downstream_types)
{
  for (const negotiated_interfaces::msg::SupportedType & type : downstream_types.supported_types) {
    std::string key_name = generate_key(type.ros_type_name, type.supported_type_name);

    if (downstream_key_to_supported_types_.count(key_name) == 0) {
      // The type to be removed was not in the map, so just skip it.
      continue;
    }

    downstream_key_to_supported_types_.erase(key_name);
  }
}

const std::unordered_map<std::string, NegotiatedSubscription::SupportedTypeInfo> &
NegotiatedSubscription::get_supported_types() const
{
  return key_to_supported_types_;
}

const negotiated_interfaces::msg::NegotiatedTopicInfo &
NegotiatedSubscription::get_existing_topic_info() const
{
  return existing_topic_info_;
}

const std::unordered_map<std::string, std::vector<NegotiatedSubscription::SupportedTypeInfo>> &
NegotiatedSubscription::get_downstream_key_to_supported_types() const
{
  return downstream_key_to_supported_types_;
}

}  // namespace negotiated
