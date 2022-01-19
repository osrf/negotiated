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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/preferences.hpp"

namespace negotiated
{

class NegotiatedSubscriber
{
public:
  template<
    typename CallbackT
  >
  explicit NegotiatedSubscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    CallbackT && callback,
    rclcpp::QoS final_qos = rclcpp::QoS(10))
  {
    auto sub_cb = [this, node, callback, final_qos](const negotiated_interfaces::msg::NewTopicInfo & msg)
      {
        RCLCPP_INFO(node->get_logger(), "Creating subscription to %s", msg.name.c_str());
        this->subscription_ = node->create_subscription<std_msgs::msg::Empty>(
          msg.name, final_qos, callback);
      };

    neg_subscription_ = node->create_subscription<negotiated_interfaces::msg::NewTopicInfo>(
      topic_name, rclcpp::QoS(10), sub_cb);

    preferences_pub_ = node->create_publisher<negotiated_interfaces::msg::Preferences>(
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

private:
  rclcpp::Subscription<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  rclcpp::Publisher<negotiated_interfaces::msg::Preferences>::SharedPtr preferences_pub_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIBER_HPP_
