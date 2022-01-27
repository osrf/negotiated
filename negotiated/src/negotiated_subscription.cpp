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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_subscription.hpp"
#include "negotiated/supported_type_map.hpp"

namespace negotiated
{

NegotiatedSubscription::NegotiatedSubscription(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  rclcpp::QoS final_qos)
: node_(node),
  topic_name_(topic_name)
{
  auto sub_cb =
    [this, node, final_qos](const negotiated_interfaces::msg::NewTopicInfo & msg)
    {
      std::string new_topic_ros_type_name = msg.ros_type_name;

      auto serialized_cb =
        [this, node, new_topic_ros_type_name](std::shared_ptr<rclcpp::SerializedMessage> msg)
        {
          supported_type_map_.dispatch_msg(new_topic_ros_type_name, msg);
        };

      this->subscription_ = node->create_generic_subscription(
        msg.topic_name, msg.ros_type_name, final_qos, serialized_cb);
    };

  neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
    topic_name, rclcpp::QoS(10), sub_cb);
}

void NegotiatedSubscription::start()
{
  // TODO(clalancette): Is this the topic name we want to use?
  supported_types_pub_ = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    topic_name_ + "/supported_types",
    rclcpp::QoS(100).transient_local());

  supported_types_pub_->publish(supported_type_map_.get_types());
}

}  // namespace negotiated
