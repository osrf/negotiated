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

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_publisher.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  const rclcpp::QoS final_qos)
: node_(node),
  topic_name_(topic_name),
  final_qos_(final_qos)
{
  negotiated_subscriber_type_gids_ = std::make_shared<std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, negotiated_interfaces::msg::SupportedTypes>>();

  neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NewTopicInfo>(
    topic_name_, rclcpp::QoS(10));

  graph_event_ = node_->get_graph_event();

  graph_change_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&NegotiatedPublisher::timer_callback, this));
}

void NegotiatedPublisher::timer_callback()
{
  // What we are doing here is checking the graph for any changes.
  // If the graph has changed, then we iterate over all of the publishers on
  // the "/supported_types" topic, which will tell us which NegotiatedSubscribers
  // to this topic are still around.  We save each of those off into a new map,
  // and replace the existing map if needed.
  //
  // If any NegotiatedSubscribers did disappear, there are two different ways we can handle it:
  // 1.  Always renegotiate, as we may be able to get something more efficient
  // 2.  Don't renegotiate, as we should just let the system continue working
  //
  // We probably want to eventually make this configurable, but for now we don't renegotiate

  node_->wait_for_graph_change(graph_event_, std::chrono::milliseconds(0));
  if (graph_event_->check_and_clear()) {
    auto new_negotiated_subscriber_gids = std::make_shared<std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, negotiated_interfaces::msg::SupportedTypes>>();
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph = node_->get_node_graph_interface();
    std::vector<rclcpp::TopicEndpointInfo> endpoints = node_graph->get_publishers_info_by_topic(topic_name_ + "/supported_types");

    for (const rclcpp::TopicEndpointInfo & endpoint : endpoints) {
      if (endpoint.endpoint_type() != rclcpp::EndpointType::Publisher) {
        // This should never happen, but just be safe
        continue;
      }

      // We only want to add GIDs to the new map if they were already in the existing map.
      // That way we avoid potential race conditions where the graph contains the information,
      // but we have not yet gotten a publication of supported types from it.
      if (negotiated_subscriber_type_gids_->count(endpoint.endpoint_gid()) > 0) {
        negotiated_interfaces::msg::SupportedTypes old_type = negotiated_subscriber_type_gids_->at(endpoint.endpoint_gid());
        new_negotiated_subscriber_gids->emplace(endpoint.endpoint_gid(), old_type);
      }
    }

    {
      std::lock_guard<std::mutex> lg(negotiated_subscriber_type_mutex_);
      negotiated_subscriber_type_gids_ = new_negotiated_subscriber_gids;
    }
  }
}

void NegotiatedPublisher::start()
{
  auto neg_cb = [this](const negotiated_interfaces::msg::SupportedTypes & supported_types, const rclcpp::MessageInfo & msg_info)
    {
      for (const negotiated_interfaces::msg::SupportedType & type :
        supported_types.supported_types)
      {
        RCLCPP_INFO(
          node_->get_logger(), "Adding supported_types %s -> %f",
          type.name.c_str(), type.weight);
        subscription_names_to_weights_[type.ros_type_name].push_back(type.weight);
      }

      std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid_key;
      std::copy(std::begin(msg_info.get_rmw_message_info().publisher_gid.data), std::end(msg_info.get_rmw_message_info().publisher_gid.data), std::begin(gid_key));

      {
        std::lock_guard<std::mutex> lg(negotiated_subscriber_type_mutex_);
        negotiated_subscriber_type_gids_->emplace(gid_key, supported_types);
      }

      negotiate();
    };

  std::string supported_type_name = topic_name_ + "/supported_types";
  supported_types_sub_ = node_->create_subscription<negotiated_interfaces::msg::SupportedTypes>(
    supported_type_name, rclcpp::QoS(100).transient_local(), neg_cb);
}

void NegotiatedPublisher::negotiate()
{
  RCLCPP_INFO(node_->get_logger(), "Negotiating");

  if (subscription_names_to_weights_.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Skipping negotiation because of empty subscription supported types");
    return;
  }

  if (supported_type_map_.get_types().supported_types.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Skipping negotiation because of empty publisher supported types");
    return;
  }

  auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();

  // TODO(clalancette): This is a stupid negotiation algorithm that just computes the
  // highest weight.  Needs work.
  double max_weight = 0.0;
  for (const negotiated_interfaces::msg::SupportedType & pub_type :
    supported_type_map_.get_types().supported_types)
  {
    RCLCPP_INFO(
      node_->get_logger(), "  Saw pub supported type %s -> %f",
      pub_type.ros_type_name.c_str(), pub_type.weight);

    std::unordered_map<std::string,
      std::vector<double>>::iterator it = subscription_names_to_weights_.find(
      pub_type.ros_type_name);
    if (it != subscription_names_to_weights_.end()) {
      double current_weight = 0.0;
      for (double w : it->second) {
        current_weight += w;
      }
      current_weight += pub_type.weight;

      if (current_weight > max_weight) {
        max_weight = current_weight;
        RCLCPP_INFO(node_->get_logger(), "  Chose type %s", pub_type.ros_type_name.c_str());
        // TODO(clalancette): What if the sub names don't match the pub name?
        msg->topic_name = topic_name_ + "/" + pub_type.name;

        ros_type_name_ = pub_type.ros_type_name;
        msg->ros_type_name = ros_type_name_;

        name_ = pub_type.name;
        msg->name = name_;
      }
    }
  }

  // Now that we've run the algorithm and figured out what our actual publication
  // "type" is going to be, create the publisher and inform the subscriptions
  // the name of it.

  publisher_ = node_->create_generic_publisher(msg->topic_name, msg->ros_type_name, final_qos_);

  neg_publisher_->publish(std::move(msg));

  return;
}

}  // namespace negotiated
