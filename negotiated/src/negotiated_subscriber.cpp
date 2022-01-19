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

#include "negotiated/negotiated_subscriber.hpp"

namespace negotiated
{

NegotiatedSubscriber::NegotiatedSubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  rclcpp::QoS final_qos)
: node_(node)
{
  auto user_cb = [this](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      RCLCPP_INFO(this->node_->get_logger(), "User callback");
    };

  auto sub_cb = [this, final_qos, user_cb](const negotiated_interfaces::msg::NewTopicInfo & msg)
    {
      RCLCPP_INFO(this->node_->get_logger(), "Creating subscription to %s", msg.name.c_str());
      this->subscription_ = this->node_->create_subscription<std_msgs::msg::Empty>(
        msg.name, final_qos, user_cb);
    };

  neg_subscription_ = node_->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
    topic_name, rclcpp::QoS(10), sub_cb);

  preferences_pub_ = node_->create_publisher<negotiated_interfaces::msg::Preferences>(
    topic_name + "_preferences",
    rclcpp::QoS(100).transient_local());

  auto prefs = std::make_unique<negotiated_interfaces::msg::Preferences>();

  negotiated_interfaces::msg::Preference pref_a;
  pref_a.name = "a";
  pref_a.weight = 1.0;
  prefs->preferences.push_back(pref_a);

  negotiated_interfaces::msg::Preference pref_b;
  pref_b.name = "b";
  pref_b.weight = 1.0;
  prefs->preferences.push_back(pref_b);

  negotiated_interfaces::msg::Preference pref_c;
  pref_c.name = "c";
  pref_c.weight = 1.0;
  prefs->preferences.push_back(pref_c);

  preferences_pub_->publish(std::move(prefs));
}

}  // namespace negotiated
