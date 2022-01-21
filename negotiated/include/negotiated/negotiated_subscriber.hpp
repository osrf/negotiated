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
#include <typeindex>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

class SupportedTypeMap final
{
public:
  template<typename SupportedMessageT>
  void add_to_map(const std::string & ros_type_name, const std::string & name, double weight)
  {
    name_to_supported_types_.emplace(typeid(SupportedMessageT), negotiated_interfaces::msg::SupportedType());
    name_to_supported_types_[typeid(SupportedMessageT)].ros_type_name = ros_type_name;
    name_to_supported_types_[typeid(SupportedMessageT)].name = name;
    name_to_supported_types_[typeid(SupportedMessageT)].weight = weight;
  }

  negotiated_interfaces::msg::SupportedTypes get() const
  {
    auto ret = negotiated_interfaces::msg::SupportedTypes();
    for (const std::pair<std::type_index, negotiated_interfaces::msg::SupportedType> & pair : name_to_supported_types_) {
      ret.supported_types.push_back(pair.second);
    }

    return ret;
  }

private:
  std::unordered_map<std::type_index, negotiated_interfaces::msg::SupportedType> name_to_supported_types_;
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

    supported_types_pub_->publish(supported_type_map.get());
  }

private:
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
