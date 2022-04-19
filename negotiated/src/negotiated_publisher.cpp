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

#include "negotiated/combinations.hpp"
#include "negotiated/negotiated_publisher.hpp"
#include "negotiated/negotiated_subscription.hpp"

namespace negotiated
{

namespace detail
{

std::string generate_key(
  const std::string & ros_type_name,
  const std::string & supported_type_name)
{
  return ros_type_name + "+" + supported_type_name;
}

std::vector<negotiated_interfaces::msg::SupportedType> default_negotiation_callback(
  const std::map<detail::PublisherGid, std::vector<std::string>> & negotiated_sub_gid_to_keys,
  const std::map<std::string, detail::SupportedTypeInfo> & key_to_supported_types,
  const std::unordered_set<std::shared_ptr<UpstreamNegotiatedSubscriptionHandle>>
  & upstream_negotiated_subscriptions,
  size_t maximum_solutions)
{
  // What the negotiation algorithm does is to try to find the minimum number of publishers with
  // the maximum amount of weight to satisfy all of the subscriptions.  This is approximately
  // equivalent to the Cutting-stock problem (https://en.wikipedia.org/wiki/Cutting_stock_problem).
  // To do this, we examine all combinations at every level (a level being the number of publishers
  // to choose), finding the highest weight one.  If there is at least one at that level, we stop
  // processing.  If there are no solutions at that level, we increment the number of publishers to
  // choose by one and try again at the next level.  If we exhaust all levels, then we have failed
  // to find a match and negotiation fails.
  //
  // Some examples will help illustrate the process.
  //
  // Scenario 1:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that all 3 subscribers support one ros_type/supported_type_name combination,
  //    and that combination (F1) is the same across all 3.
  //   Finally assume that the publisher also supports the same ros_type/supported_type_name
  //    combination (F1)
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    Since all of the subscriptions and the publisher support F1, then a "level 1" solution
  //    exists, and the algorithm chooses that.
  //
  // Scenario 2:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that S1 and S2 support one ros_type/supported_type_name combination (F1), and
  //    S3 supports a different ros_type/supported_type_name combination (F2).
  //   Finally assume that the publisher supports both F1 and F2 ros_type/supported_type_name
  //    combinations.
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    The publisher and S1 and S2 support F1, but S3 does not, so there is no one publisher
  //    solution.  Next the algorithm tries all combinations of 2 publisher solutions.  In this
  //    case we can make 2 publishers, one to satisify F1 and F2, so that algorithm chooses that.
  //
  // Scenario 3:
  //   Assume there are 3 subscribers, S1, S2, and S3.
  //   Further assume that S1 and S2 support one ros_type/supported_type_name combination (F1), and
  //    S3 supports a different ros_type/supported_type_name combination (F2).
  //   Finally assume that the publisher supports only the F1 ros_type/supported_type_name
  //     combinations.
  //   When negotiation happens the publisher will try to find a solution that can satisify all of
  //    S1, S2, and S3.  It starts by examining all of the solutions that involve one publisher.
  //    The publisher and S1 and S2 support F1, but S3 does not, so there is no one publisher
  //    solution.  Next the algorithm tries all combinations of 2 publisher solutions.  Since the
  //    publisher doesn't support F2, there are no 2 publisher solutions.  Finally the algorithm
  //    tries the 3 publisher solution, but since the publisher doesn't support F2 this can't
  //    work either.  So the negotiation fails in this case.

  std::vector<negotiated_interfaces::msg::SupportedType> matched_subs;

  // If there are upstream subscriptions that we should wait on before negotiating with our
  // downstream subscriptions, we'll discover it here.
  std::map<std::string, SupportedTypeInfo> upstream_filtered_supported_types;
  if (upstream_negotiated_subscriptions.size() > 0) {
    bool all_negotiated = true;
    for (const std::shared_ptr<UpstreamNegotiatedSubscriptionHandle> & handle :
      upstream_negotiated_subscriptions)
    {
      negotiated_interfaces::msg::NegotiatedTopicsInfo topics_info =
        handle->subscription->get_negotiated_topics_info();
      if (!topics_info.success || topics_info.negotiated_topics.size() == 0) {
        all_negotiated = false;
        break;
      }

      for (const negotiated_interfaces::msg::NegotiatedTopicInfo & topic_info :
        topics_info.negotiated_topics)
      {
        std::string key = detail::generate_key(
          topic_info.ros_type_name,
          topic_info.supported_type_name);

        if (key_to_supported_types.count(key) > 0) {
          upstream_filtered_supported_types[key] = key_to_supported_types.at(key);
        }
      }
    }

    if (!all_negotiated) {
      return matched_subs;
    }
  } else {
    upstream_filtered_supported_types = key_to_supported_types;
  }

  std::set<detail::PublisherGid> gid_set;
  for (const std::pair<detail::PublisherGid,
    std::vector<std::string>> & gid : negotiated_sub_gid_to_keys)
  {
    gid_set.insert(gid.first);
  }

  std::vector<std::string> keys;
  std::vector<detail::SupportedTypeInfo> compatible_supported_types;
  for (const std::pair<const std::string,
    detail::SupportedTypeInfo> & supported_info : upstream_filtered_supported_types)
  {
    keys.push_back(supported_info.first);
    if (supported_info.second.is_compat) {
      compatible_supported_types.push_back(supported_info.second);
    }
  }

  for (size_t i = 1; i <= upstream_filtered_supported_types.size(); ++i) {
    double max_weight = 0.0;

    auto check_combination =
      [&upstream_filtered_supported_types = std::as_const(upstream_filtered_supported_types),
        & gid_set = std::as_const(gid_set),
        & compatible_supported_types = std::as_const(compatible_supported_types),
        & negotiated_sub_gid_to_keys = std::as_const(negotiated_sub_gid_to_keys),
        &max_weight,
        &matched_subs](
      std::vector<std::string>::iterator first,
      std::vector<std::string>::iterator last) -> bool
      {
        std::set<detail::PublisherGid> gids_needed = gid_set;

        double sum_of_weights = 0.0;

        for (std::vector<std::string>::iterator it = first; it != last; ++it) {
          // The iterator should *always* be available in the upstream_filtered_supported_types
          // map, since we are iterating over that same map.  But we use .at just
          // to be safe.
          detail::SupportedTypeInfo supported_type_info = upstream_filtered_supported_types.at(*it);

          for (const std::pair<detail::PublisherGid,
            double> gid_to_weight : supported_type_info.gid_to_weight)
          {
            sum_of_weights += gid_to_weight.second;

            gids_needed.erase(gid_to_weight.first);
          }
        }

        std::vector<negotiated_interfaces::msg::SupportedType> compatible_subs;
        if (!compatible_supported_types.empty()) {
          // We've removed all of the ones we could above in this iteration.  Now we go through
          // the remaining list of GIDs, seeing if any of the "compatible" supported types satisfy
          // the requirements.
          for (std::set<detail::PublisherGid>::const_iterator it = gids_needed.begin();
            it != gids_needed.end(); )
          {
            const std::vector<std::string> key_list = negotiated_sub_gid_to_keys.at(*it);

            bool found_key = false;
            for (const detail::SupportedTypeInfo & compat_info : compatible_supported_types) {
              std::string key = detail::generate_key(
                compat_info.ros_type_name,
                compat_info.supported_type_name);
              if (std::find(key_list.begin(), key_list.end(), key) != key_list.end()) {
                negotiated_interfaces::msg::SupportedType match;
                match.ros_type_name = compat_info.ros_type_name;
                match.supported_type_name = compat_info.supported_type_name;
                compatible_subs.push_back(match);

                found_key = true;
                break;
              }
            }

            if (found_key) {
              it = gids_needed.erase(it);
            } else {
              ++it;
            }
          }
        }

        if (gids_needed.empty()) {
          // Hooray!  We found a solution at this level.  We don't interrupt processing at this
          // level because there may be another combination that is more favorable, but we know
          // we don't need to descend to further levels.

          if (sum_of_weights > max_weight) {
            max_weight = sum_of_weights;

            matched_subs.clear();
            matched_subs = compatible_subs;
            for (std::vector<std::string>::iterator it = first; it != last; ++it) {
              detail::SupportedTypeInfo supported_type_info =
                upstream_filtered_supported_types.at(*it);
              negotiated_interfaces::msg::SupportedType match;
              match.ros_type_name = supported_type_info.ros_type_name;
              match.supported_type_name = supported_type_info.supported_type_name;
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

    if (i == maximum_solutions) {
      break;
    }
  }

  return matched_subs;
}

void default_update_downstream_cb(
  const std::map<std::string, detail::SupportedTypeInfo> & key_to_supported_types,
  const std::shared_ptr<std::map<detail::PublisherGid,
  std::vector<std::string>>> & negotiated_subscription_type_gids,
  const std::unordered_set<std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>> &
  upstream_negotiated_subscriptions,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_add,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_remove,
  PublisherGid gid_key)
{
  (void)key_to_supported_types;
  (void)negotiated_subscription_type_gids;

  for (const std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle> & handle :
    upstream_negotiated_subscriptions)
  {
    handle->subscription->update_downstream_supported_types(
      downstream_types_to_add,
      downstream_types_to_remove,
      gid_key);
  }
}

}  // namespace detail

NegotiatedPublisher::NegotiatedPublisher(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
  const std::string & topic_name,
  const NegotiatedPublisherOptions & negotiated_pub_options)
: node_parameters_(node_parameters),
  node_topics_(node_topics),
  node_logging_(node_logging),
  node_graph_(node_graph),
  node_base_(node_base),
  node_timers_(node_timers),
  topic_name_(topic_name),
  negotiated_pub_options_(negotiated_pub_options)
{
  if (negotiated_pub_options_.maximum_negotiated_solutions == 0) {
    throw std::invalid_argument("maximum_negotiated_solutions must be larger than 0");
  }

  negotiated_subscription_type_gids_ = std::make_shared<std::map<detail::PublisherGid,
      std::vector<std::string>>>();

  negotiated_publisher_ =
    rclcpp::create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    node_parameters_,
    node_topics_,
    topic_name_,
    rclcpp::QoS(10));

  graph_event_ = node_graph_->get_graph_event();

  graph_change_timer_ = rclcpp::create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NegotiatedPublisher::graph_change_timer_callback, this),
    nullptr,
    node_base_.get(),
    node_timers_.get());
}

NegotiatedPublisher::~NegotiatedPublisher()
{
  if (upstream_negotiated_subscriptions_.size() > 0) {
    negotiated_interfaces::msg::SupportedTypes downstream_types;
    for (const std::pair<std::string, detail::SupportedTypeInfo> & type : key_to_supported_types_) {
      negotiated_interfaces::msg::SupportedType downstream_type;
      downstream_type.ros_type_name = type.second.ros_type_name;
      downstream_type.supported_type_name = type.second.supported_type_name;
      downstream_types.supported_types.push_back(downstream_type);
    }
    for (const std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle> & handle :
      upstream_negotiated_subscriptions_)
    {
      handle->subscription->remove_all_downstream_supported_types(downstream_types);
    }
  }

  // TODO(clalancette): I think we also need to remove upstream callbacks
}

void NegotiatedPublisher::graph_change_timer_callback()
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
  // Which one we do is controlled by the negotiate_on_subscription_removal option; by default,
  // we renegotiate.

  node_graph_->wait_for_graph_change(graph_event_, std::chrono::milliseconds(0));
  if (!graph_event_->check_and_clear()) {
    return;
  }

  auto new_negotiated_subscription_gids = std::make_shared<std::map<detail::PublisherGid,
      std::vector<std::string>>>();
  std::vector<rclcpp::TopicEndpointInfo> endpoints = node_graph_->get_publishers_info_by_topic(
    negotiated_publisher_->get_topic_name() + std::string("/_supported_types"));

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

  for (const std::pair<detail::PublisherGid,
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
          node_logging_->get_logger(),
          "Could not find key in supported_types, this shouldn't happen");
        continue;
      }

      if (key_to_supported_types_[key].gid_to_weight.count(gid_to_key.first) == 0) {
        // This should also never happen, but just be careful.
        RCLCPP_INFO(
          node_logging_->get_logger(),
          "Could not find gid in supported_types, this shouldn't happen");
        continue;
      }

      key_to_supported_types_[key].gid_to_weight.erase(gid_to_key.first);
    }
  }

  negotiated_subscription_type_gids_ = new_negotiated_subscription_gids;

  if (different_maps && negotiated_pub_options_.negotiate_on_subscription_removal) {
    negotiate();
  }
}

void NegotiatedPublisher::negotiate_on_upstream_success()
{
  negotiate();
}

std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>
NegotiatedPublisher::add_upstream_negotiated_subscription(
  std::shared_ptr<negotiated::NegotiatedSubscription> subscription)
{
  auto it = std::find_if(
    upstream_negotiated_subscriptions_.begin(),
    upstream_negotiated_subscriptions_.end(),
    [&subscription =
    std::as_const(subscription)](
      const std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle> & check_handle) {
      return subscription == check_handle->subscription;
    });

  if (it != upstream_negotiated_subscriptions_.end()) {
    RCLCPP_WARN(node_logging_->get_logger(), "Adding duplicate upstream negotiated subscription!");
  }

  auto upstream_handle = std::make_shared<detail::UpstreamNegotiatedSubscriptionHandle>();
  upstream_handle->subscription = subscription;
  upstream_handle->handle =
    subscription->add_after_subscription_callback(
    std::bind(&NegotiatedPublisher::negotiate_on_upstream_success, this));

  upstream_negotiated_subscriptions_.insert(upstream_handle);

  return upstream_handle;
}

void NegotiatedPublisher::remove_upstream_negotiated_subscription(
  const detail::UpstreamNegotiatedSubscriptionHandle * const handle)
{
  auto it = std::find_if(
    upstream_negotiated_subscriptions_.begin(),
    upstream_negotiated_subscriptions_.end(),
    [handle](const std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle> & check_handle) {
      return handle == check_handle.get();
    });
  if (it != upstream_negotiated_subscriptions_.end()) {
    (*it)->subscription->remove_after_subscription_callback((*it)->handle.get());
    upstream_negotiated_subscriptions_.erase(it);
  } else {
    RCLCPP_WARN(
      node_logging_->get_logger(),
      "Attempted to remove upstream negotiated subscription that didn't exist");
  }
}

void NegotiatedPublisher::supported_types_cb(
  const negotiated_interfaces::msg::SupportedTypes & supported_types,
  const rclcpp::MessageInfo & msg_info)
{
  detail::PublisherGid gid_key;
  std::copy(
    std::begin(msg_info.get_rmw_message_info().publisher_gid.data),
    std::end(msg_info.get_rmw_message_info().publisher_gid.data),
    std::begin(gid_key));

  negotiated_interfaces::msg::SupportedTypes downstream_types_to_add;
  negotiated_interfaces::msg::SupportedTypes downstream_types_to_remove;

  if (negotiated_subscription_type_gids_->count(gid_key) > 0) {
    // This NegotiatedSubscription has already given us previous types that we need to forget about.
    for (const std::string & key : negotiated_subscription_type_gids_->at(gid_key)) {
      if (key_to_supported_types_.count(key) == 0) {
        // Odd, but just continue on.
        continue;
      }

      if (key_to_supported_types_[key].gid_to_weight.count(gid_key) == 0) {
        // Odd, but just continue on.
        continue;
      }

      key_to_supported_types_[key].gid_to_weight.erase(gid_key);

      negotiated_interfaces::msg::SupportedType downstream_type_to_remove;
      downstream_type_to_remove.ros_type_name = key_to_supported_types_[key].ros_type_name;
      downstream_type_to_remove.supported_type_name =
        key_to_supported_types_[key].supported_type_name;
      downstream_types_to_remove.supported_types.push_back(downstream_type_to_remove);

      // In theory, we should check to see if the gid_to_weight map size dropped to zero, and if so,
      // remove that type from the key_to_supported_types_ map completely.  However, we only ever
      // store information about NegotiatedSubscriptions that match something in this
      // NegotiatedPublisher, so in practice that list will never be of size 0.
    }

    negotiated_subscription_type_gids_->erase(gid_key);
  }

  std::vector<std::string> key_list;

  for (const negotiated_interfaces::msg::SupportedType & supported_type :
    supported_types.supported_types)
  {
    std::string key = detail::generate_key(
      supported_type.ros_type_name,
      supported_type.supported_type_name);
    if (key_to_supported_types_.count(key) == 0) {
      // This key is not something the publisher supports, so we can ignore it completely
      continue;
    }

    key_to_supported_types_[key].gid_to_weight[gid_key] = supported_type.weight;

    key_list.push_back(key);

    if (upstream_negotiated_subscriptions_.size() > 0) {
      negotiated_interfaces::msg::SupportedType downstream_type_to_add;
      downstream_type_to_add.ros_type_name = key_to_supported_types_[key].ros_type_name;
      downstream_type_to_add.supported_type_name = key_to_supported_types_[key].supported_type_name;
      downstream_type_to_add.weight = key_to_supported_types_[key].gid_to_weight[gid_key];
      downstream_types_to_add.supported_types.push_back(downstream_type_to_add);
    }
  }

  // Only add a new subscription to the GID map if any of the keys matched.
  if (!key_list.empty()) {
    std::lock_guard<std::mutex> lg(negotiated_subscription_type_mutex_);
    negotiated_subscription_type_gids_->emplace(gid_key, key_list);
  }

  negotiated_pub_options_.update_downstream_cb(
    key_to_supported_types_,
    negotiated_subscription_type_gids_,
    upstream_negotiated_subscriptions_,
    downstream_types_to_add,
    downstream_types_to_remove,
    gid_key);

  if (negotiated_pub_options_.negotiate_on_subscription_add) {
    negotiate();
  }
}

void NegotiatedPublisher::start()
{
  std::string supported_type_name =
    negotiated_publisher_->get_topic_name() + std::string("/_supported_types");
  supported_types_sub_ = rclcpp::create_subscription<negotiated_interfaces::msg::SupportedTypes>(
    node_parameters_,
    node_topics_,
    supported_type_name,
    rclcpp::QoS(100).transient_local(),
    std::bind(
      &NegotiatedPublisher::supported_types_cb, this, std::placeholders::_1,
      std::placeholders::_2));
}

void NegotiatedPublisher::stop()
{
  supported_types_sub_.reset();
  for (std::pair<const std::string,
    detail::SupportedTypeInfo> & supported_info : key_to_supported_types_)
  {
    if (!supported_info.second.is_compat) {
      supported_info.second.publisher = nullptr;
    }
  }
  negotiated_subscription_type_gids_->clear();
}

void NegotiatedPublisher::negotiate()
{
  RCLCPP_INFO(node_logging_->get_logger(), "Negotiating");

  if (key_to_supported_types_.empty()) {
    RCLCPP_INFO(
      node_logging_->get_logger(),
      "Skipping negotiation because of empty publisher supported types");
    return;
  }

  std::vector<negotiated_interfaces::msg::SupportedType> matched_subs;

  if (!negotiated_subscription_type_gids_->empty()) {
    matched_subs = negotiated_pub_options_.negotiation_cb(
      *negotiated_subscription_type_gids_,
      key_to_supported_types_,
      upstream_negotiated_subscriptions_,
      negotiated_pub_options_.maximum_negotiated_solutions);
  }

  negotiated_topics_info_.negotiated_topics.clear();

  if (matched_subs.empty()) {
    // We couldn't find any match, so don't setup anything
    RCLCPP_INFO(node_logging_->get_logger(), "Could not negotiate");
    if (negotiated_pub_options_.disconnect_publishers_on_failure) {
      for (std::pair<const std::string,
        detail::SupportedTypeInfo> & supported_info : key_to_supported_types_)
      {
        if (!supported_info.second.is_compat) {
          supported_info.second.publisher = nullptr;
        }
      }
    }

    negotiated_topics_info_.success = false;
  } else {
    // Now that we've run the algorithm and figured out what our actual publication
    // "type" is going to be, create the publisher(s) and inform the subscriptions
    // the name(s) of them.

    // Note that we only recreate the publishers if the new list of publishers is different from
    // the old list, e.g. it is different than the last time we negotiated.  This keeps us from
    // unnecessarily tearing down and recreating the publishers if the set is going to be exactly
    // the same as last time.  In all cases, though, we send out the information to the
    // subscriptions so they can act accordingly (even new ones).

    negotiated_topics_info_.success = true;

    std::set<std::string> keys_to_preserve;
    for (const negotiated_interfaces::msg::SupportedType & type : matched_subs) {
      std::string key = detail::generate_key(type.ros_type_name, type.supported_type_name);

      if (key_to_supported_types_.count(key) == 0) {
        // Somehow the negotiation algorithm returned a non-existent key to us.  Log
        // it and omit it from the list we send to the subscriptions.
        RCLCPP_WARN(
          node_logging_->get_logger(),
          "Negotiation algorithm returned bogus %s:%s that is not a supported type",
          type.ros_type_name.c_str(),
          type.supported_type_name.c_str());
        continue;
      }

      detail::SupportedTypeInfo & supported_type_info = key_to_supported_types_[key];

      keys_to_preserve.insert(key);
      if (supported_type_info.publisher == nullptr) {
        // We need to create this publisher.
        std::string topic_name =
          negotiated_publisher_->get_topic_name() + std::string("/") + type.supported_type_name;
        supported_type_info.publisher = supported_type_info.pub_factory(topic_name);
      }

      negotiated_interfaces::msg::NegotiatedTopicInfo info;
      info.ros_type_name = type.ros_type_name;
      info.supported_type_name = type.supported_type_name;
      info.topic_name = supported_type_info.publisher->get_topic_name();

      negotiated_topics_info_.negotiated_topics.push_back(info);
    }

    // Now go through and remove any publishers that are no longer needed.
    for (std::pair<const std::string,
      detail::SupportedTypeInfo> & supported_info : key_to_supported_types_)
    {
      if (keys_to_preserve.count(supported_info.first) == 0) {
        if (!supported_info.second.is_compat) {
          supported_info.second.publisher = nullptr;
        }
      }
    }

    if (negotiated_pub_options_.successful_negotiation_cb != nullptr) {
      negotiated_pub_options_.successful_negotiation_cb(negotiated_topics_info_);
    }
  }

  negotiated_publisher_->publish(negotiated_topics_info_);
}

const std::map<std::string, detail::SupportedTypeInfo> &
NegotiatedPublisher::get_supported_types() const
{
  return key_to_supported_types_;
}

const negotiated_interfaces::msg::NegotiatedTopicsInfo &
NegotiatedPublisher::get_negotiated_topics_info() const
{
  return negotiated_topics_info_;
}

}  // namespace negotiated
