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

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

struct SupportedTypeInfo final
{
  negotiated_interfaces::msg::SupportedType supported_type;
  std::function<rclcpp::SubscriptionBase::SharedPtr(const std::string &)> sub_factory;
};

class NegotiatedSubscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedSubscription)

  explicit NegotiatedSubscription(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name);

  template<typename T, typename CallbackT>
  void add_supported_callback(
    double weight,
    CallbackT callback,
    const rclcpp::QoS & qos)
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key_name = generate_key(ros_type_name, T::format_match);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, SupportedTypeInfo());
    key_to_supported_types_[key_name].supported_type.ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type.format_match = T::format_match;
    key_to_supported_types_[key_name].supported_type.weight = weight;

    auto factory =
      [this, callback, qos](const std::string & topic_name) -> rclcpp::SubscriptionBase::SharedPtr
      {
        return node_->create_subscription<typename T::MsgT>(topic_name, qos, callback);
      };

    key_to_supported_types_[key_name].sub_factory = factory;
  }

  void start();

private:
  std::string generate_key(const std::string & ros_type_name, const std::string & format_match);

  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, SupportedTypeInfo> key_to_supported_types_;
  rclcpp::Subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr
    neg_subscription_;
  std::shared_ptr<rclcpp::SubscriptionBase> subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
  std::string ros_type_name_;
  std::string format_match_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
