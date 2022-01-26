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

#ifndef NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
#define NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

class MessageContainerBase
{
public:
  virtual std::shared_ptr<void> get_msg_ptr() = 0;
};

template<typename MessageT>
class MessageContainer final : public MessageContainerBase
{
public:
  MessageContainer()
  {
    message_storage = std::make_shared<MessageT>();
  }

  std::shared_ptr<void> get_msg_ptr() override
  {
    return message_storage;
  }

private:
  std::shared_ptr<MessageT> message_storage;
};

struct SupportedTypeInfo final
{
  negotiated_interfaces::msg::SupportedType supported_type;
  std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> asc;
  std::shared_ptr<rclcpp::SerializationBase> serializer;
  std::shared_ptr<MessageContainerBase> message_container;
};

class SupportedTypeMap final
{
public:
  template<typename MessageT, typename CallbackT>
  void add_to_map(
    const std::string & ros_type_name,
    const std::string & name,
    double weight,
    CallbackT && callback)
  {
    // TODO(clalancette): What if the supported type is already in the map?
    name_to_supported_types_.emplace(ros_type_name, SupportedTypeInfo());
    name_to_supported_types_[ros_type_name].supported_type.ros_type_name = ros_type_name;
    name_to_supported_types_[ros_type_name].supported_type.name = name;
    name_to_supported_types_[ros_type_name].supported_type.weight = weight;
    auto asc = std::make_shared<rclcpp::AnySubscriptionCallback<MessageT>>();
    asc->set(callback);
    name_to_supported_types_[ros_type_name].asc = asc;
    name_to_supported_types_[ros_type_name].serializer =
      std::make_shared<rclcpp::Serialization<MessageT>>();
    name_to_supported_types_[ros_type_name].message_container =
      std::make_shared<MessageContainer<MessageT>>();
  }

  negotiated_interfaces::msg::SupportedTypes get_types() const;

  void dispatch_msg(
    const std::string & ros_type_name,
    std::shared_ptr<rclcpp::SerializedMessage> msg) const;

private:
  std::unordered_map<std::string, SupportedTypeInfo> name_to_supported_types_;
};

class NegotiatedSubscription
{
public:
  explicit NegotiatedSubscription(
    rclcpp::Node::SharedPtr node,
    const SupportedTypeMap & supported_type_map,
    const std::string & topic_name,
    rclcpp::QoS final_qos = rclcpp::QoS(10));

private:
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
