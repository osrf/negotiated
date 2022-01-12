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

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_interfaces/node_graph.hpp"
#include "std_msgs/msg/empty.hpp"

#include "negotiated_interfaces/srv/negotiated_preferences.hpp"

#include "negotiated/negotiated_publisher.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(rclcpp::Node::SharedPtr node, const std::string & topic_name)
  : topic_name_(topic_name),
    node_(node)
{
  // TODO(clalancette): can we just use node->create_publisher() ?
  neg_publisher_ = rclcpp::create_publisher<std_msgs::msg::Empty>(node_, topic_name_, rclcpp::QoS(10));
}

bool NegotiatedPublisher::negotiate()
{
  std::vector<std::string> preferences;

  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph = node_->get_node_graph_interface();
  std::vector<rclcpp::TopicEndpointInfo> sub_info_list = node_graph->get_subscriptions_info_by_topic(topic_name_);

  // TODO(clalancette): we could probably significantly speed this up by
  // checking which negotiated subscriptions are ready now, sending the request
  // to all of those, and then periodically waiting on the rest until they ar
  // ready.
  for (const rclcpp::TopicEndpointInfo & info : sub_info_list) {
    std::string name = info.node_name();
    std::string ns = info.node_namespace();

    RCLCPP_INFO(node_->get_logger(), "Attempting to negotiate with %s/%s", name.c_str(), ns.c_str());

    rclcpp::Client<negotiated_interfaces::srv::NegotiatedPreferences>::SharedPtr client = node_->create_client<negotiated_interfaces::srv::NegotiatedPreferences>(name + "/negotiation_service");

    auto request = std::make_shared<negotiated_interfaces::srv::NegotiatedPreferences::Request>();
    request->command = "negotiate";

    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service");
        return false;
      }

      // TODO(clalancette): we should eventually give up on this one and go to
      // the next one.
    }

    auto result = client->async_send_request(request);
    // TODO(clalancette): timeout?
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
      auto result_pointer = result.get();
      RCLCPP_INFO(node_->get_logger(), "Result: %s", result_pointer->preferences.c_str());
      // TODO(clalancette): if the NegotiatedSubscriber gave us empty preferences,
      // we should probably not respond to them; something is probably wrong
      std::string pref = result_pointer->preferences;
      preferences.push_back(pref);
    } else {
      RCLCPP_INFO(node_->get_logger(), "Failed to call service with command");
      // TODO(clalancette): this means that if any of the negotiated subscribers
      // goes away before we can perform negotiation, the whole thing fails.
      // We should probably instead return a list of those that failed.
      return false;
    }

  }

  // TODO(clalancette): run the algorithm to choose the type

  // Now that we've run the algorithm and figured out what our actual publication
  // "type" is going to be, create the publisher and inform the subscriptions
  // the name of it.

  // TODO(clalancette): can we just use node->create_publisher() ?
  std::string new_topic_name = topic_name_ + "/yuv422";
  publisher_ = rclcpp::create_publisher<std_msgs::msg::Empty>(node_, new_topic_name, rclcpp::QoS(10));

  // TODO(clalancette): we could probably significantly speed this up by
  // checking which negotiated subscriptions are ready now, sending the request
  // to all of those, and then periodically waiting on the rest until they ar
  // ready.
  for (const rclcpp::TopicEndpointInfo & info : sub_info_list) {
    std::string name = info.node_name();
    std::string ns = info.node_namespace();

    RCLCPP_INFO(node_->get_logger(), "Setting topic name to %s/%s", name.c_str(), ns.c_str());

    rclcpp::Client<negotiated_interfaces::srv::NegotiatedPreferences>::SharedPtr client = node_->create_client<negotiated_interfaces::srv::NegotiatedPreferences>(name + "/negotiation_service");

    auto request = std::make_shared<negotiated_interfaces::srv::NegotiatedPreferences::Request>();
    request->command = "set_negotiated_name";
    request->name = new_topic_name;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service");
        return false;
      }

      // TODO(clalancette): we should eventually give up on this one and go to
      // the next one.
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    } else {
      RCLCPP_INFO(node_->get_logger(), "Failed to call service with name");
      // TODO(clalancette): this means that if any of the negotiated subscribers
      // goes away before we can perform negotiation, the whole thing fails.
      // We should probably instead return a list of those that failed.
      return false;
    }
  }

  return true;
}

}  // namespace negotiated
