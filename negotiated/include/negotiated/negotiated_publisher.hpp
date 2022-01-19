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

#ifndef NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
#define NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/new_topic_info.hpp"
#include "negotiated_interfaces/msg/preference.hpp"
#include "negotiated_interfaces/msg/preferences.hpp"

namespace negotiated
{

template<typename MessageT, typename Alloc = std::allocator<void>>
class NegotiatedPublisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedPublisher)

  using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;

  explicit NegotiatedPublisher(
    rclcpp::Node::SharedPtr node, const std::string & topic_name,
    const rclcpp::QoS final_qos = rclcpp::QoS(10))
  : node_(node),
    topic_name_(topic_name),
    final_qos_(final_qos)
  {
    neg_publisher_ = node_->create_publisher<negotiated_interfaces::msg::NewTopicInfo>(
      topic_name_, rclcpp::QoS(10));

    auto neg_cb = [this](const negotiated_interfaces::msg::Preferences & prefs)
      {
        // TODO(clalancette): We have to consider renegotiation here
        for (const negotiated_interfaces::msg::Preference & pref : prefs.preferences) {
          fprintf(stderr, "Adding preferences %s -> %f\n", pref.name.c_str(), pref.weight);
          this->preferences_.push_back(pref);
        }
      };

    std::string pref_name = topic_name_ + "/preferences";
    fprintf(stderr, "About to subscribe to %s\n", pref_name.c_str());
    pref_sub_ = node_->create_subscription<negotiated_interfaces::msg::Preferences>(
      pref_name, rclcpp::QoS(100).transient_local(), neg_cb);
  }

  bool negotiate()
  {
    for (const negotiated_interfaces::msg::Preference & pref : preferences_) {
      fprintf(stderr, "Saw preference %s -> %f\n", pref.name.c_str(), pref.weight);
    }

    // TODO(clalancette): run the algorithm to choose the type

    // Now that we've run the algorithm and figured out what our actual publication
    // "type" is going to be, create the publisher and inform the subscriptions
    // the name of it.

    std::string new_topic_name = topic_name_ + "/yuv422";
    publisher_ = node_->create_publisher<MessageT>(new_topic_name, final_qos_);

    auto msg = std::make_unique<negotiated_interfaces::msg::NewTopicInfo>();
    msg->name = new_topic_name;
    neg_publisher_->publish(std::move(msg));

    return true;
  }

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    publisher_->publish(std::move(msg));
  }

  void
  publish(const MessageT & msg)
  {
    publisher_->publish(msg);
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_name_;
  rclcpp::QoS final_qos_;
  rclcpp::Publisher<negotiated_interfaces::msg::NewTopicInfo>::SharedPtr neg_publisher_;
  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  rclcpp::Subscription<negotiated_interfaces::msg::Preferences>::SharedPtr pref_sub_;
  std::vector<negotiated_interfaces::msg::Preference> preferences_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
