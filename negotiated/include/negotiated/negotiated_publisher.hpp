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

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
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
  std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)> pub_factory;
};

class NegotiatedPublisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedPublisher)

  explicit NegotiatedPublisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name);

  template<typename T>
  void add_supported_type(double weight, const rclcpp::QoS & qos)
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
      [this, qos](const std::string & topic_name) -> rclcpp::PublisherBase::SharedPtr
      {
        return node_->create_publisher<typename T::MsgT>(topic_name, qos);
      };

    key_to_supported_types_[key_name].pub_factory = factory;
  }

  void start();

  template<typename T>
  bool type_was_negotiated()
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    return ros_type_name_ == ros_type_name && format_match_ == T::format_match;
  }

  template<typename MessageT>
  void publish(const MessageT & msg)
  {
    if (publisher_ == nullptr) {
      RCLCPP_INFO(node_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    // TODO(clalancette): What if this is a publish for a type we didn't negotiate for?

    auto pub = static_cast<rclcpp::Publisher<MessageT> *>(publisher_.get());
    pub->publish(msg);
  }

private:
  void negotiate();

  void timer_callback();

  std::string generate_key(const std::string & ros_type_name, const std::string & format_match);

  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, SupportedTypeInfo> key_to_supported_types_;
  std::string topic_name_;
  std::string ros_type_name_;
  std::string format_match_;
  rclcpp::Publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr neg_publisher_;
  std::shared_ptr<rclcpp::PublisherBase> publisher_{nullptr};
  rclcpp::Subscription<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_sub_;
  rclcpp::TimerBase::SharedPtr graph_change_timer_;
  rclcpp::Event::SharedPtr graph_event_;
  std::mutex negotiated_subscription_type_mutex_;
  std::shared_ptr<std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>,
    negotiated_interfaces::msg::SupportedTypes>> negotiated_subscription_type_gids_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
