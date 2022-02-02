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
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include "rclcpp/node_interfaces/node_graph.hpp"
#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topic_info.hpp"
#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_publisher.hpp"

#include "combinations.hpp"

namespace negotiated
{
NegotiatedPublisher::NegotiatedPublisher(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  const NegotiatedPublisherOptions & neg_pub_options)
: node_(node),
  topic_name_(topic_name),
  neg_pub_options_(neg_pub_options)
{
  negotiated_subscription_type_gids_ = std::make_shared<std::map<PublisherGid,
      std::vector<std::string>>>();

  neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    topic_name_, rclcpp::QoS(10));

  graph_event_ = node_->get_graph_event();

  graph_change_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NegotiatedPublisher::timer_callback, this));
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
  if (!graph_event_->check_and_clear()) {
    return;
  }

  auto new_negotiated_subscription_gids = std::make_shared<std::map<PublisherGid,
      std::vector<std::string>>>();
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph =
    node_->get_node_graph_interface();
  std::vector<rclcpp::TopicEndpointInfo> endpoints = node_graph->get_publishers_info_by_topic(
    topic_name_ + "/supported_types");

  // We need to hold the lock across this entire operation
  std::lock_guard<std::mutex> lg(negotiated_subscription_type_mutex_);

  for (const rclcpp::TopicEndpointInfo & endpoint : endpoints) {
    if (endpoint.endpoint_type() != rclcpp::EndpointType::Publisher) {
      // This should never happen, but just be safe
      continue;
    }

    // We only want to add GIDs to the new map if they were already in the existing map.
    // That way we avoid potential race conditions where the graph contains the information,
    // but we have not yet gotten a publication of supported types from it.
    if (negotiated_subscription_type_gids_->count(endpoint.endpoint_gid()) > 0) {
      std::vector<std::string> old_type =
        negotiated_subscription_type_gids_->at(endpoint.endpoint_gid());
      new_negotiated_subscription_gids->emplace(endpoint.endpoint_gid(), old_type);
    }
  }

  // OK, now that we have built up a new map, we need to go through the new and old map together.
  // This is so we can reduce counts and weights on entries that have gone away.
  bool different_maps = false;

  for (const std::pair<PublisherGid,
    std::vector<std::string>> & gid_to_key : *negotiated_subscription_type_gids_)
  {
    if (new_negotiated_subscription_gids->count(gid_to_key.first) > 0) {
      // The key from the old is in the new one, so no need to do any work here
      continue;
    }

    different_maps = true;

    // The key from the old is *not* in the new one, so we need to go through and remove the
    // weights from the key_to_supported_types_ map.

    for (const std::string & key : gid_to_key.second) {
      if (key_to_supported_types_.count(key) == 0) {
        // This should never happen, but just be careful
        RCLCPP_INFO(
          node_->get_logger(), "Could not find key in supported_types, this shouldn't happen");
        continue;
      }

      if (key_to_supported_types_[key].gid_to_weight.count(gid_to_key.first) == 0) {
        // This should also never happen, but just be careful.
        RCLCPP_INFO(
          node_->get_logger(), "Could not find gid in supported_types, this shouldn't happen");
        continue;
      }

      key_to_supported_types_[key].gid_to_weight.erase(gid_to_key.first);
    }
  }

  negotiated_subscription_type_gids_ = new_negotiated_subscription_gids;

  if (different_maps && neg_pub_options_.negotiate_on_subscription_removal) {
    negotiate();
  }
}

std::string NegotiatedPublisher::generate_key(
  const std::string & ros_type_name,
  const std::string & format_match)
{
  return ros_type_name + "+" + format_match;
}

void NegotiatedPublisher::start()
{
  auto neg_cb =
    [this](const negotiated_interfaces::msg::SupportedTypes & supported_types,
      const rclcpp::MessageInfo & msg_info)
    {
      PublisherGid gid_key;
      std::copy(
        std::begin(msg_info.get_rmw_message_info().publisher_gid.data),
        std::end(msg_info.get_rmw_message_info().publisher_gid.data),
        std::begin(gid_key));

      std::vector<std::string> key_list;

      for (const negotiated_interfaces::msg::SupportedType & supported_type :
        supported_types.supported_types)
      {
        std::string key = generate_key(supported_type.ros_type_name, supported_type.format_match);
        if (key_to_supported_types_.count(key) == 0) {
          // This key is not something the publisher supports, so we can ignore it completely
          continue;
        }

        key_to_supported_types_[key].gid_to_weight[gid_key] = supported_type.weight;

        key_list.push_back(key);
      }

      // Only add a new subscription to the GID map if any of the keys matched.
      if (!key_list.empty()) {
        std::lock_guard<std::mutex> lg(negotiated_subscription_type_mutex_);
        negotiated_subscription_type_gids_->emplace(gid_key, key_list);
      }

      if (neg_pub_options_.negotiate_on_subscription_add) {
        negotiate();
      }
    };

  std::string supported_type_name = topic_name_ + "/supported_types";
  supported_types_sub_ = node_->create_subscription<negotiated_interfaces::msg::SupportedTypes>(
    supported_type_name, rclcpp::QoS(100).transient_local(), neg_cb);
}

void NegotiatedPublisher::negotiate()
{
  RCLCPP_INFO(node_->get_logger(), "Negotiating");

  if (negotiated_subscription_type_gids_->empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Skipping negotiation because of empty subscription supported types");
    return;
  }

  if (key_to_supported_types_.empty()) {
    RCLCPP_INFO(
      node_->get_logger(), "Skipping negotiation because of empty publisher supported types");
    return;
  }

  // The negotiation algorithm starts here.  In short, what it does is to try to find the minimum
  // number of publishers with the maximum amount of weight to satisifes all of the subscriptions.
  // This is approximately equivalent to the Cutting-stock problem
  // (https://en.wikipedia.org/wiki/Cutting_stock_problem).  To do this, we examine all combinations
  // at every level (a level being the number of publishers to choose), finding the highest weight
  // one.  If there is at least one at that level, we stop processing.  If there are no solutions
  // at that level, we increment the number of publishers to choose by one and try again at the
  // next level.  If we exhaust all levels, then we have failed to find a match and negotiation
  // fails.
  //
  // Some examples will help illustrate the process.
  //
  // Scenario 1:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that all 3 subscribers support one ros_type/format_match combination, and
  //    that combination (F1) is the same across all 3.
  //   Finally assume that the publisher also supports the same ros_type/format_match
  //    combination (F1)
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    Since all of the subscriptions and the publisher support F1, then a "level 1" solution
  //    exists, and the algorithm chooses that.
  //
  // Scenario 2:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that S1 and S2 support one ros_type/format_match combination (F1), and
  //    S3 supports a different ros_type/format_match combination (F2).
  //   Finally assume that the publisher supports both F1 and F2 ros_type/format_match combinations.
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    The publisher and S1 and S2 support F1, but S3 does not, so there is no one publisher
  //    solution.  Next the algorithm tries all combinations of 2 publisher solutions.  In this
  //    case we can make 2 publishers, one to satisify F1 and F2, so that algorithm chooses that.
  //
  // Scenario 3:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that S1 and S2 support one ros_type/format_match combination (F1), and
  //    S3 supports a different ros_type/format_match combination (F2).
  //   Finally assume that the publisher supports only the F1 ros_type/format_match combinations.
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    The publisher and S1 and S2 support F1, but S3 does not, so there is no one publisher
  //    solution.  Next the algorithm tries all combinations of 2 publisher solutions.  Since the
  //    publisher doesn't support F2, there are no 2 publisher solutions.  Finally the algorithm
  //    tries the 3 publisher solution, but since the publisher doesn't support F2 this can't
  //    work either.  So the negotiation fails in this case.

  std::vector<std::string> keys;
  for (const std::pair<const std::string,
    SupportedTypeInfo> & supported_info : key_to_supported_types_)
  {
    keys.push_back(supported_info.first);
  }

  std::vector<negotiated_interfaces::msg::SupportedType> matched_subs;
  for (size_t i = 1; i <= key_to_supported_types_.size(); ++i) {
    double max_weight = 0.0;

    auto check_combination =
      [this, &max_weight, &matched_subs](std::vector<std::string>::iterator first,
        std::vector<std::string>::iterator last) -> bool
      {
        std::set<PublisherGid> gids_needed;
        for (const std::pair<PublisherGid,
          std::vector<std::string>> & gid : *negotiated_subscription_type_gids_)
        {
          gids_needed.insert(gid.first);
        }

        double sum_of_weights = 0.0;

        for (std::vector<std::string>::iterator it = first; it != last; ++it) {
          // The iterator should *always* be available in the key_to_supported_types_
          // map, since we are iterating over that same map.  But we use .at just
          // to be safe.
          SupportedTypeInfo supported_type_info = key_to_supported_types_.at(*it);

          for (const std::pair<PublisherGid,
            double> gid_to_weight : supported_type_info.gid_to_weight)
          {
            sum_of_weights += gid_to_weight.second;

            gids_needed.erase(gid_to_weight.first);
          }
        }

        if (gids_needed.empty()) {
          // Hooray!  We found a solution at this level.  We don't interrupt processing at this
          // level because there may be another combination that is more favorable, but we know
          // we don't need to descend to further levels.

          if (sum_of_weights > max_weight) {
            max_weight = sum_of_weights;

            matched_subs.clear();
            for (std::vector<std::string>::iterator it = first; it != last; ++it) {
              SupportedTypeInfo supported_type_info = key_to_supported_types_.at(*it);
              negotiated_interfaces::msg::SupportedType match;
              match.ros_type_name = supported_type_info.ros_type_name;
              match.format_match = supported_type_info.format_match;
              matched_subs.push_back(match);
            }
          }
        }

        return false;
      };

    for_each_combination(keys.begin(), keys.begin() + i, keys.end(), check_combination);

    if (!matched_subs.empty()) {
      break;
    }
  }

  auto msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();

  if (matched_subs.empty()) {
    // We couldn't find any match, so don't setup anything
    RCLCPP_INFO(node_->get_logger(), "Could not negotiate");
    key_to_publisher_.clear();

    msg->success = false;
  } else {
    // Now that we've run the algorithm and figured out what our actual publication
    // "type" is going to be, create the publisher(s) and inform the subscriptions
    // the name(s) of them.

    // Note that we only recreate the publishers if the new list of publishers is different from
    // the old list, e.g. it is different than the last time we negotiated.  This keeps us from
    // unnecessarily tearing down and recreating the publishers if the set is going to be exactly
    // the same as last time.  In all cases, though, we send out the information to the
    // subscriptions so they can act accordingly (even new ones).

    msg->success = true;

    std::set<std::string> keys_to_preserve;
    for (const negotiated_interfaces::msg::SupportedType & type : matched_subs) {
      negotiated_interfaces::msg::NegotiatedTopicInfo info;
      info.ros_type_name = type.ros_type_name;
      info.format_match = type.format_match;
      info.topic_name = topic_name_ + "/" + type.format_match;
      msg->negotiated_topics.push_back(info);

      std::string key = generate_key(type.ros_type_name, type.format_match);
      keys_to_preserve.insert(key);
      if (key_to_publisher_.count(key) == 0) {
        // This particular subscription is not yet in the map, so we need to create it.
        auto pub_factory = key_to_supported_types_.at(key).pub_factory;
        key_to_publisher_[key] = pub_factory(info.topic_name);
      }
    }

    // Now go through and remove any publishers that are no longer needed.
    for (std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>::iterator it =
      key_to_publisher_.begin(); it != key_to_publisher_.end(); )
    {
      if (keys_to_preserve.count(it->first) == 0) {
        key_to_publisher_.erase(it++);
      } else {
        ++it;
      }
    }
  }

  neg_publisher_->publish(std::move(msg));
}

}  // namespace negotiated
