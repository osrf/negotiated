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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

struct NegotiatedPublisherOptions final
{
  bool negotiate_on_subscription_removal{true};
  bool negotiate_on_subscription_add{true};
};

class NegotiatedPublisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedPublisher)

  explicit NegotiatedPublisher(
    rclcpp::Node * node,
    const std::string & topic_name,
    const NegotiatedPublisherOptions & neg_pub_options = NegotiatedPublisherOptions());

  template<typename T>
  void add_supported_type(
    double weight,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions())
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key_name = generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::runtime_error("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, SupportedTypeInfo());

    auto factory =
      [this, qos, options](const std::string & topic_name) -> rclcpp::PublisherBase::SharedPtr
      {
        return rclcpp::create_publisher<typename T::MsgT>(
          node_parameters_,
          node_topics_,
          topic_name,
          qos,
          options);
      };
    key_to_supported_types_[key_name].pub_factory = factory;

    PublisherGid gid{0};
    key_to_supported_types_[key_name].gid_to_weight[gid] = weight;
    key_to_supported_types_[key_name].ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type_name = T::supported_type_name;
  }

  void start();

  void negotiate();

  template<typename T>
  bool type_was_negotiated()
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key = generate_key(ros_type_name, T::supported_type_name);
    return key_to_publisher_.count(key) > 0;
  }

  template<typename T, typename MessageT>
  void publish(const MessageT & msg)
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key = generate_key(ros_type_name, T::supported_type_name);

    if (key_to_publisher_.count(key) == 0) {
      RCLCPP_INFO(node_logging_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    std::shared_ptr<rclcpp::PublisherBase> publisher_ = key_to_publisher_[key];

    auto pub = static_cast<rclcpp::Publisher<MessageT> *>(publisher_.get());
    pub->publish(msg);
  }

  template<typename T, typename MessageT>
  void publish(std::unique_ptr<MessageT> msg)
  {
    std::string ros_type_name = rosidl_generator_traits::name<typename T::MsgT>();
    std::string key = generate_key(ros_type_name, T::supported_type_name);

    if (key_to_publisher_.count(key) == 0) {
      RCLCPP_INFO(node_logging_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    std::shared_ptr<rclcpp::PublisherBase> publisher_ = key_to_publisher_[key];

    auto pub = static_cast<rclcpp::Publisher<MessageT> *>(publisher_.get());
    pub->publish(std::move(msg));
  }

private:
  using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

  struct SupportedTypeInfo final
  {
    std::map<PublisherGid, double> gid_to_weight;
    std::string ros_type_name;
    std::string supported_type_name;
    std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)> pub_factory;
  };

  void timer_callback();

  std::string generate_key(
    const std::string & ros_type_name,
    const std::string & supported_type_name);

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  std::string topic_name_;
  NegotiatedPublisherOptions neg_pub_options_;

  std::map<std::string, SupportedTypeInfo> key_to_supported_types_;
  rclcpp::Publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr neg_publisher_;
  std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> key_to_publisher_;
  rclcpp::Subscription<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_sub_;
  rclcpp::TimerBase::SharedPtr graph_change_timer_;
  rclcpp::Event::SharedPtr graph_event_;
  std::mutex negotiated_subscription_type_mutex_;
  std::shared_ptr<std::map<PublisherGid,
    std::vector<std::string>>> negotiated_subscription_type_gids_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
