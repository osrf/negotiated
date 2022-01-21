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

#ifndef NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
#define NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

template<typename MessageT, typename Alloc = std::allocator<void>>
class NegotiatedPublisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedPublisher)

  using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;

  explicit NegotiatedPublisher(
    rclcpp::Node::SharedPtr node,
    const negotiated_interfaces::msg::SupportedTypes & supported_types,
    const std::string & topic_name,
    const rclcpp::QoS final_qos = rclcpp::QoS(10))
  : node_(node),
    pub_supported_types_(supported_types),
    topic_name_(topic_name),
    final_qos_(final_qos)
  {
    neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NewTopicInfo>(
      topic_name_, rclcpp::QoS(10));

    auto neg_cb = [this](const negotiated_interfaces::msg::SupportedTypes & supported_types)
      {
        // TODO(clalancette): We have to consider renegotiation here
        for (const negotiated_interfaces::msg::SupportedType & type :
          supported_types.supported_types)
        {
          fprintf(stderr, "Adding supported_types %s -> %f\n", type.name.c_str(), type.weight);
          this->sub_supported_types_.push_back(type);
        }
      };

    std::string supported_type_name = topic_name_ + "/supported_types";
    fprintf(stderr, "About to subscribe to %s\n", supported_type_name.c_str());
    supported_types_sub_ = node_->create_subscription<negotiated_interfaces::msg::SupportedTypes>(
      supported_type_name, rclcpp::QoS(100).transient_local(), neg_cb);
  }

  bool negotiate()
  {
    for (const negotiated_interfaces::msg::SupportedType & type : sub_supported_types_) {
      fprintf(stderr, "Saw supported type %s -> %f\n", type.name.c_str(), type.weight);
    }

    // TODO(clalancette): run the algorithm to choose the type

    // TODO(clalancette): What happens if the subscription supported_types are empty?
    // TODO(clalancette): What happens if the publisher supported_types are empty?

    // Now that we've run the algorithm and figured out what our actual publication
    // "type" is going to be, create the publisher and inform the subscriptions
    // the name of it.

    std::string new_topic_name = topic_name_ + "/yuv422";
    publisher_ = node_->create_publisher<MessageT>(new_topic_name, final_qos_);

    auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();
    msg->name = new_topic_name;
    neg_publisher_->publish(std::move(msg));

    return true;
  }

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    // TODO(clalancette): Only publish here if negotiation is successful
    publisher_->publish(std::move(msg));
  }

  void
  publish(const MessageT & msg)
  {
    // TODO(clalancette): Only publish here if negotiation is successful
    publisher_->publish(msg);
  }

private:
  rclcpp::Node::SharedPtr node_;
  negotiated_interfaces::msg::SupportedTypes pub_supported_types_;
  std::string topic_name_;
  rclcpp::QoS final_qos_;
  rclcpp::Publisher<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_publisher_;
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  rclcpp::Subscription<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_sub_;
  std::vector<negotiated_interfaces::msg::SupportedType> sub_supported_types_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
