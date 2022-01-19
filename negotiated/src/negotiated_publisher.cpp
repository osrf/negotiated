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
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"

#include "negotiated/negotiated_publisher.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(rclcpp::Node::SharedPtr node, const std::string & topic_name)
  : topic_name_(topic_name),
    node_(node)
{
  neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NewTopicInfo>(topic_name_, rclcpp::QoS(10));

  auto user_cb = [this](const std_msgs::msg::String & pref)
  {
    // TODO(clalancette): We have to consider renegotiation here
    fprintf(stderr, "Saw data %s\n", pref.data.c_str());
    preferences_.push_back(pref.data);
  };

  std::string pref_name = topic_name_ + "_preferences";
  fprintf(stderr, "About to subscribe to %s\n", pref_name.c_str());
  pref_sub_ = node_->create_subscription<std_msgs::msg::String>(pref_name, rclcpp::QoS(100).transient_local(), user_cb);
}

bool NegotiatedPublisher::negotiate()
{
  for (const std::string & pref : preferences_) {
    fprintf(stderr, "Saw preferences %s\n", pref.c_str());
  }

  // TODO(clalancette): run the algorithm to choose the type

  // Now that we've run the algorithm and figured out what our actual publication
  // "type" is going to be, create the publisher and inform the subscriptions
  // the name of it.

  std::string new_topic_name = topic_name_ + "/yuv422";
  publisher_ = node_->create_publisher<std_msgs::msg::Empty>(new_topic_name, rclcpp::QoS(10));

  auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();
  msg->name = new_topic_name;
  neg_publisher_->publish(std::move(msg));

  return true;
}

}  // namespace negotiated
