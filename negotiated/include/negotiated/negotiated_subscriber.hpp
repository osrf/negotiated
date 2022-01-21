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

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

class SupportedTypeMap final
{
public:
  void add_to_map(const negotiated_interfaces::msg::SupportedTypes & supported_types)
  {
    type_to_supported_types_["foo"] = supported_types;
  }

  negotiated_interfaces::msg::SupportedTypes get(const std::string & key) const
  {
    return type_to_supported_types_.at(key);
  }

private:
  std::unordered_map<std::string, negotiated_interfaces::msg::SupportedTypes> type_to_supported_types_;
};

template<typename MessageT>
class NegotiatedSubscriber
{
public:
  template<typename CallbackT>
  explicit NegotiatedSubscriber(
    rclcpp::Node::SharedPtr node,
    const SupportedTypeMap & supported_type_map,
    const std::string & topic_name,
    CallbackT && callback,
    rclcpp::QoS final_qos = rclcpp::QoS(10))
  {
    auto sub_cb =
      [this, node, callback, final_qos](const negotiated_interfaces::msg::NewTopicInfo & msg)
      {
        RCLCPP_INFO(node->get_logger(), "Creating subscription to %s", msg.name.c_str());
        this->subscription_ = node->create_subscription<MessageT>(
          msg.name, final_qos, callback);
      };

    neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
      topic_name, rclcpp::QoS(10), sub_cb);

    // TODO(clalancette): Is this the topic name we want to use?
    supported_types_pub_ = node->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
      topic_name + "/supported_types",
      rclcpp::QoS(100).transient_local());

    supported_types_pub_->publish(supported_type_map.get("foo"));
  }

private:
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
