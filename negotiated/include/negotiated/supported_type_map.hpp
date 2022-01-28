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

#ifndef NEGOTIATED__SUPPORTED_TYPE_MAP_HPP_
#define NEGOTIATED__SUPPORTED_TYPE_MAP_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

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
  std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> asc{nullptr};
  std::shared_ptr<rclcpp::SerializationBase> serializer;
  std::shared_ptr<MessageContainerBase> message_container;
};

class SupportedTypeMap final
{
public:
  template<typename T, typename CallbackT>
  void add_supported_callback(double weight, CallbackT && callback)
  {
    std::string key_name = T::ros_type + "+" + T::name;
    if (name_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    add_common_info<T>(key_name, weight);

    auto asc = std::make_shared<rclcpp::AnySubscriptionCallback<typename T::MsgT>>();
    asc->set(callback);
    name_to_supported_types_[key_name].asc = asc;

    name_to_supported_types_[key_name].message_container =
      std::make_shared<MessageContainer<typename T::MsgT>>();
  }

  template<typename T>
  void add_supported_info(double weight)
  {
    std::string key_name = T::ros_type + "+" + T::name;
    if (name_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    add_common_info<T>(key_name, weight);
  }

  negotiated_interfaces::msg::SupportedTypes get_types() const;

  std::shared_ptr<rclcpp::SerializationBase> get_serializer(
    const std::string & ros_type_name,
    const std::string & name) const;

  std::shared_ptr<MessageContainerBase> get_msg_container(
    const std::string & ros_type_name,
    const std::string & name) const;

  std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> get_any_subscription_callback(
    const std::string & ros_type_name,
    const std::string & name) const;

private:
  template<typename T>
  void add_common_info(const std::string & key_name, double weight)
  {
    name_to_supported_types_.emplace(key_name, SupportedTypeInfo());
    name_to_supported_types_[key_name].supported_type.ros_type_name = T::ros_type;
    name_to_supported_types_[key_name].supported_type.name = T::name;
    name_to_supported_types_[key_name].supported_type.weight = weight;
    name_to_supported_types_[key_name].serializer =
      std::make_shared<rclcpp::Serialization<typename T::MsgT>>();
  }

  std::unordered_map<std::string, SupportedTypeInfo> name_to_supported_types_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__SUPPORTED_TYPE_MAP_HPP_
