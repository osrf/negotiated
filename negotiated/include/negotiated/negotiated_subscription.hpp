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

#include "negotiated_interfaces/msg/negotiated_topic_info.hpp"
#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

namespace detail
{

negotiated_interfaces::msg::NegotiatedTopicInfo default_negotiate_cb(
  const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg);

}  // namespace detail

struct NegotiatedSubscriptionOptions
{
  bool disconnect_on_negotiation_failure{true};

  std::function<negotiated_interfaces::msg::NegotiatedTopicInfo(
      const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
      const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)> negotiate_cb{
    detail::default_negotiate_cb};
};

class NegotiatedSubscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedSubscription)

  explicit NegotiatedSubscription(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_,
    const std::string & topic_name,
    const NegotiatedSubscriptionOptions & negotiated_sub_options = NegotiatedSubscriptionOptions());

  template<typename NodeT>
  explicit NegotiatedSubscription(
    NodeT & node,
    const std::string & topic_name,
    const NegotiatedSubscriptionOptions & negotiated_sub_options = NegotiatedSubscriptionOptions())
  : NegotiatedSubscription(
      node.get_node_parameters_interface(),
      node.get_node_topics_interface(),
      topic_name,
      negotiated_sub_options)
  {
  }

  template<typename T, typename CallbackT>
  void add_supported_callback(
    double weight,
    const rclcpp::QoS & qos,
    CallbackT callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
  {
    if (T::supported_type_name.empty()) {
      throw std::runtime_error("The supported_type_name cannot be empty");
    }

    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();

    std::string key_name = generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, SupportedTypeInfo());
    key_to_supported_types_[key_name].supported_type.ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type.supported_type_name = T::supported_type_name;
    key_to_supported_types_[key_name].supported_type.weight = weight;

    auto factory =
      [this, qos, callback,
        options](const std::string & topic_name) -> rclcpp::SubscriptionBase::SharedPtr
      {
        return rclcpp::create_subscription<typename T::MsgT>(
          node_parameters_,
          node_topics_,
          topic_name,
          qos,
          callback,
          options);
      };

    key_to_supported_types_[key_name].sub_factory = factory;
  }

  template<typename T>
  void remove_supported_callback()
  {
    if (T::supported_type_name.empty()) {
      throw std::runtime_error("The supported_type_name cannot be empty");
    }

    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();

    std::string key_name = generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) == 0) {
      throw std::runtime_error("Specified key does not exist");
    }

    key_to_supported_types_.erase(key_name);
  }

  void start();

  size_t get_negotiated_topic_publisher_count() const;

  size_t get_data_topic_publisher_count() const;

private:
  struct SupportedTypeInfo final
  {
    negotiated_interfaces::msg::SupportedType supported_type;
    std::function<rclcpp::SubscriptionBase::SharedPtr(const std::string &)> sub_factory;
  };

  std::string generate_key(
    const std::string & ros_type_name,
    const std::string & supported_type_name);

  void topicsInfoCb(const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg);

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  NegotiatedSubscriptionOptions negotiated_sub_options_;

  std::unordered_map<std::string, SupportedTypeInfo> key_to_supported_types_;
  rclcpp::Subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr
    negotiated_subscription_;
  std::shared_ptr<rclcpp::SubscriptionBase> subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;

  negotiated_interfaces::msg::NegotiatedTopicInfo existing_topic_info_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
