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

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/srv/negotiated_preferences.hpp"

#include "negotiated/negotiated_subscriber.hpp"

namespace negotiated
{

NegotiatedSubscriber::NegotiatedSubscriber(rclcpp::Node::SharedPtr node, const std::string & topic_name)
  : node_(node)
{
  auto user_cb = [this](const std_msgs::msg::Empty & msg)
  {
    (void)msg;
    RCLCPP_INFO(this->node_->get_logger(), "User callback");
  };

  auto sub_cb = [this, user_cb](const negotiated_interfaces::msg::NewTopicInfo & msg)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Creating subscription to %s", msg.name.c_str());
    this->subscription_ = this->node_->create_subscription<std_msgs::msg::Empty>(msg.name, rclcpp::QoS(10), user_cb);
  };

  // TODO(clalancette): can we just use node->create_subscription() here?
  neg_subscription_ = rclcpp::create_subscription<negotiated_interfaces::msg::NewTopicInfo>(node_, topic_name, rclcpp::QoS(10), sub_cb, rclcpp::SubscriptionOptions());

  auto srv_cb = [](const negotiated_interfaces::srv::NegotiatedPreferences::Request::SharedPtr req,
                   negotiated_interfaces::srv::NegotiatedPreferences::Response::SharedPtr resp)
  {
    (void)req;

    // TODO(clalancette): this should be given to us by the user somehow
    resp->preferences = "a,b,c";
  };

  negotiation_srv_ = rclcpp::create_service<negotiated_interfaces::srv::NegotiatedPreferences>(node->get_node_base_interface(), node->get_node_services_interface(), std::string(node_->get_name()) + "/" + topic_name + "/negotiation_service", srv_cb, rmw_qos_profile_services_default, nullptr);
}

}  // namespace negotiated
