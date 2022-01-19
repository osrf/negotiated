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
#include "negotiated_interfaces/msg/preference.hpp"
#include "negotiated_interfaces/msg/preferences.hpp"

#include "negotiated/negotiated_publisher.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(
  rclcpp::Node::SharedPtr node, const std::string & topic_name, const rclcpp::QoS final_qos)
: node_(node),
  topic_name_(topic_name),
  final_qos_(final_qos)
{
  neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NewTopicInfo>(
    topic_name_, rclcpp::QoS(10));

  auto user_cb = [this](const negotiated_interfaces::msg::Preferences & prefs)
    {
      // TODO(clalancette): We have to consider renegotiation here
      for (const negotiated_interfaces::msg::Preference & pref : prefs.preferences) {
        fprintf(stderr, "Adding preferences %s -> %f\n", pref.name.c_str(), pref.weight);
        this->preferences_.push_back(pref);
      }
    };

  std::string pref_name = topic_name_ + "_preferences";
  fprintf(stderr, "About to subscribe to %s\n", pref_name.c_str());
  pref_sub_ = node_->create_subscription<negotiated_interfaces::msg::Preferences>(
    pref_name, rclcpp::QoS(100).transient_local(), user_cb);
}

bool NegotiatedPublisher::negotiate()
{
  for (const negotiated_interfaces::msg::Preference & pref : preferences_) {
    fprintf(stderr, "Saw preference %s -> %f\n", pref.name.c_str(), pref.weight);
  }

  // TODO(clalancette): run the algorithm to choose the type

  // Now that we've run the algorithm and figured out what our actual publication
  // "type" is going to be, create the publisher and inform the subscriptions
  // the name of it.

  std::string new_topic_name = topic_name_ + "/yuv422";
  publisher_ = node_->create_publisher<std_msgs::msg::Empty>(new_topic_name, final_qos_);

  auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();
  msg->name = new_topic_name;
  neg_publisher_->publish(std::move(msg));

  return true;
}

}  // namespace negotiated
