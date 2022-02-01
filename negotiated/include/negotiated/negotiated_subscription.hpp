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

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/supported_type_map.hpp"

namespace negotiated
{

class NegotiatedSubscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedSubscription)

  explicit NegotiatedSubscription(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name);

  template<typename T, typename CallbackT>
  void add_supported_callback(double weight, CallbackT callback, const rclcpp::QoS & qos)
  {
    supported_type_map_.add_supported_callback<T, CallbackT>(node_, weight, callback, qos);
  }

  void start();

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_name_;
  SupportedTypeMap supported_type_map_;
  rclcpp::Subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr neg_subscription_;
  std::shared_ptr<rclcpp::SubscriptionBase> subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
  std::string ros_type_name_;
  std::string format_match_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
