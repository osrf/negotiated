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

#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

struct SupportedTypeInfo final
{
  negotiated_interfaces::msg::SupportedType supported_type;
  std::function<rclcpp::SubscriptionBase::SharedPtr(const std::string &)> sub_factory;
  std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)> pub_factory;
};

class SupportedTypeMap final
{
public:
  template<typename T, typename CallbackT>
  void add_supported_callback(rclcpp::Node::SharedPtr node, double weight, CallbackT callback, const rclcpp::QoS & qos)
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key_name = ros_type_name + "+" + T::name;
    if (name_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    add_common_info<T>(key_name, ros_type_name, weight);

    auto factory =
      [node, callback, qos](const std::string & topic_name) -> rclcpp::SubscriptionBase::SharedPtr
      {
        return node->create_subscription<typename T::MsgT>(topic_name, qos, callback);
      };

    name_to_supported_types_[key_name].sub_factory = factory;
  }

  template<typename T>
  void add_supported_info(rclcpp::Node::SharedPtr node, double weight, const rclcpp::QoS & qos)
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key_name = ros_type_name + "+" + T::name;
    if (name_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    add_common_info<T>(key_name, ros_type_name, weight);

    auto factory =
      [node, qos](const std::string & topic_name) -> rclcpp::PublisherBase::SharedPtr
      {
        return node->create_publisher<typename T::MsgT>(topic_name, qos);
      };

    name_to_supported_types_[key_name].pub_factory = factory;
  }

  negotiated_interfaces::msg::SupportedTypes get_types() const;

  std::function<rclcpp::SubscriptionBase::SharedPtr(const std::string &)> get_sub_factory(
    const std::string & ros_type_name,
    const std::string & name) const;

  std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)> get_pub_factory(
    const std::string & ros_type_name,
    const std::string & name) const;

private:
  template<typename T>
  void add_common_info(const std::string & key_name, const std::string & ros_type_name, double weight)
  {
    name_to_supported_types_.emplace(key_name, SupportedTypeInfo());
    name_to_supported_types_[key_name].supported_type.ros_type_name = ros_type_name;
    name_to_supported_types_[key_name].supported_type.name = T::name;
    name_to_supported_types_[key_name].supported_type.weight = weight;
  }

  std::unordered_map<std::string, SupportedTypeInfo> name_to_supported_types_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__SUPPORTED_TYPE_MAP_HPP_
