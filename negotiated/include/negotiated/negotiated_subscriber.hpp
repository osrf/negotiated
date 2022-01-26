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

#ifndef NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
#define NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_

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

  negotiated_interfaces::msg::SupportedTypes get() const
  {
    auto ret = negotiated_interfaces::msg::SupportedTypes();
    for (const std::pair<const std::string, SupportedTypeInfo> & pair : name_to_supported_types_) {
      ret.supported_types.push_back(pair.second.supported_type);
    }

    return ret;
  }

  void dispatch_msg(
    const std::string & ros_type_name,
    std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    if (name_to_supported_types_.count(ros_type_name) == 0) {
      // We were asked to dispatch for a type that we don't have, so skip
      return;
    }

    // TODO(clalancette): This is bogus; what should we fill in?
    rclcpp::MessageInfo msg_info;

    SupportedTypeInfo type_info = name_to_supported_types_.at(ros_type_name);

    std::shared_ptr<MessageContainerBase> msg_container = type_info.message_container;
    std::shared_ptr<void> msg_ptr = msg_container->get_msg_ptr();

    std::shared_ptr<rclcpp::SerializationBase> serializer = type_info.serializer;
    serializer->deserialize_message(msg.get(), msg_ptr.get());

    std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> asc = type_info.asc;
    asc->dispatch(msg_ptr, msg_info);
  }

private:
  std::unordered_map<std::string, SupportedTypeInfo> name_to_supported_types_;
};

// TODO(clalancette): Now that this isn't templatized, we can move this to a .cpp file
class NegotiatedSubscriber
{
public:
  explicit NegotiatedSubscriber(
    rclcpp::Node::SharedPtr node,
    const SupportedTypeMap & supported_type_map,
    const std::string & topic_name,
    rclcpp::QoS final_qos = rclcpp::QoS(10))
  {
    auto sub_cb =
      [this, node, supported_type_map,
        final_qos](const negotiated_interfaces::msg::NewTopicInfo & msg)
      {
        std::string new_topic_ros_type_name = msg.ros_type_name;

        auto serialized_cb =
          [this, node, supported_type_map,
            new_topic_ros_type_name](std::shared_ptr<rclcpp::SerializedMessage> msg)
          {
            supported_type_map.dispatch_msg(new_topic_ros_type_name, msg);
          };

        this->subscription_ = node->create_generic_subscription(
          msg.topic_name, msg.ros_type_name, final_qos, serialized_cb);
      };

    neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
      topic_name, rclcpp::QoS(10), sub_cb);

    // TODO(clalancette): Is this the topic name we want to use?
    supported_types_pub_ = node->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
      topic_name + "/supported_types",
      rclcpp::QoS(100).transient_local());

    supported_types_pub_->publish(supported_type_map.get());
  }

private:
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
