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

#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

struct SupportedTypeInfo final
{
  negotiated_interfaces::msg::SupportedType supported_type;
  std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> callback;
};

class SupportedTypeMap final
{
public:
  template<typename SupportedMessageT, typename CallbackT>
  void add_to_map(const std::string & ros_type_name, const std::string & name, double weight, CallbackT && callback)
  {
    // TODO(clalancette): What if the supported type is already in the map?
    name_to_supported_types_.emplace(ros_type_name, SupportedTypeInfo());
    name_to_supported_types_[ros_type_name].supported_type.ros_type_name = ros_type_name;
    name_to_supported_types_[ros_type_name].supported_type.name = name;
    name_to_supported_types_[ros_type_name].supported_type.weight = weight;
    auto any_sub_callback = std::make_shared<rclcpp::AnySubscriptionCallback<SupportedMessageT>>();
    any_sub_callback->set(callback);
    name_to_supported_types_[ros_type_name].callback = any_sub_callback;
  }

  negotiated_interfaces::msg::SupportedTypes get() const
  {
    auto ret = negotiated_interfaces::msg::SupportedTypes();
    for (const std::pair<const std::string, SupportedTypeInfo> & pair : name_to_supported_types_) {
      ret.supported_types.push_back(pair.second.supported_type);
    }

    return ret;
  }

  std::unordered_map<std::string, SupportedTypeInfo> name_to_supported_types_;
};

// TODO(clalancette): Now that this isn't templatized, we can move this to a .cpp file
class NegotiatedSubscriber
{
public:
  explicit NegotiatedSubscriber(
    rclcpp::Node::SharedPtr node,
    const SupportedTypeMap & supported_type_map,
    const std::string & topic_name,
    rclcpp::QoS final_qos = rclcpp::QoS(10))
  {
    asc_ = std::make_shared<rclcpp::AnySubscriptionCallback<std_msgs::msg::String>>();
    asc_->set(std::bind(&NegotiatedSubscriber::string_cb, this, std::placeholders::_1));

    auto sub_cb =
      [this, node, final_qos](const negotiated_interfaces::msg::NewTopicInfo & msg)
      {
        RCLCPP_INFO(node->get_logger(), "Creating subscription to %s with type %s", msg.topic_name.c_str(), msg.ros_type_name.c_str());

        auto cb = [this, node](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          RCLCPP_INFO(node->get_logger(), "Got serialized message");
          // TODO(clalancette): This is bogus; what should we fill in?
          rclcpp::MessageInfo msg_info;

          auto string_message = std::make_shared<std_msgs::msg::String>();
          rclcpp::Serialization<std_msgs::msg::String> serializer;
          serializer.deserialize_message(msg.get(), string_message.get());

          asc_->dispatch(string_message, msg_info);
        };

        this->subscription_ = node->create_generic_subscription(msg.topic_name, msg.ros_type_name, final_qos, cb);

        //auto ts_lib = rclcpp::get_typesupport_library(
        //  msg.ros_type_name, "rosidl_typesupport_cpp");
        //const rosidl_message_type_support_t * type_support_handle = rclcpp::get_typesupport_handle(msg.ros_type_name, "rosidl_typesupport_cpp", *ts_lib);
        //std::shared_ptr<rclcpp::AnySubscriptionCallbackBase> cb = supported_type_map.name_to_supported_types_.at(msg.ros_type_name).callback;

        //rclcpp::AnySubscriptionCallback * mycb = static_cast<rclcpp::AnySubscriptionCallback<std_msgs::msg::String>>(*cb.get());

        //this->subscription_ = node->create_subscription<MessageT>(
        //  msg.topic_name, final_qos, mycb);
        //const SupportedTypeInfo & type_info = supported_type_map.name_to_supported_types_.at(typeid(std::string));
        //auto cb = std::unique_ptr<AnySubscriptionCallback<MessageT>>(static_cast<AnySubscriptionCallback<int>*>(sti.callback.release()));
        //this->subscription_ = node->create_subscription<std_msg::msg::String>(
        //  msg.name, final_qos, type_info.callback);
      };

    neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
      topic_name, rclcpp::QoS(10), sub_cb);

    // TODO(clalancette): Is this the topic name we want to use?
    supported_types_pub_ = node->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
      topic_name + "/supported_types",
      rclcpp::QoS(100).transient_local());

    supported_types_pub_->publish(supported_type_map.get());
  }

  void string_cb(const std_msgs::msg::String & msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("string_cb"), "User-like callback: %s", msg.data.c_str());
  }

private:
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;
  std::shared_ptr<rclcpp::AnySubscriptionCallback<std_msgs::msg::String>> asc_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
