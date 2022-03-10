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
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_publisher.hpp"
#include "negotiated/negotiated_subscription.hpp"

#include "example_type_info.hpp"

namespace negotiated_examples
{

class NegotiatedPubSubPairExample2 final : public rclcpp::Node
{
public:
  explicit NegotiatedPubSubPairExample2(const rclcpp::NodeOptions & options)
  : rclcpp::Node("negotiated_pub_sub_pair_example2", options)
  {
    negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(*this, "example");

    bool use_intra_process = this->declare_parameter("use_intra_process", false);
    rclcpp::SubscriptionOptions sub_options;
    if (use_intra_process) {
      sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    auto string_user_cb = [this](const std_msgs::msg::String & msg)
      {
        RCLCPP_INFO(this->get_logger(), "String user callback: %s", msg.data.c_str());
        std_msgs::msg::String new_msg;
        new_msg.data = msg.data + " intermediate";

        if (negotiated_pub1_->type_was_negotiated<negotiated_examples::StringT>()) {
          negotiated_pub1_->publish<negotiated_examples::StringT>(new_msg);
        }
        if (negotiated_pub2_->type_was_negotiated<negotiated_examples::StringT>()) {
          negotiated_pub2_->publish<negotiated_examples::StringT>(new_msg);
        }
      };

    auto string2_user_cb = [this](const std_msgs::msg::String & msg)
      {
        RCLCPP_INFO(this->get_logger(), "String2 user callback: %s", msg.data.c_str());
        std_msgs::msg::String new_msg;
        new_msg.data = msg.data + " intermediate2";

        if (negotiated_pub1_->type_was_negotiated<negotiated_examples::StringT2>()) {
          negotiated_pub1_->publish<negotiated_examples::StringT2>(new_msg);
        }
        if (negotiated_pub2_->type_was_negotiated<negotiated_examples::StringT2>()) {
          negotiated_pub2_->publish<negotiated_examples::StringT2>(new_msg);
        }
      };

    negotiated_sub_->add_supported_callback<negotiated_examples::StringT>(
      1.0,
      rclcpp::QoS(1),
      string_user_cb,
      sub_options);
    negotiated_sub_->add_supported_callback<negotiated_examples::StringT2>(
      0.5,
      rclcpp::QoS(1),
      string2_user_cb,
      sub_options);

    rclcpp::PublisherOptions pub_options;
    if (use_intra_process) {
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    negotiated_pub1_ = std::make_shared<negotiated::NegotiatedPublisher>(
      *this,
      "downstream/example");
    negotiated_pub1_->add_supported_type<negotiated_examples::StringT>(
      0.1,
      rclcpp::QoS(1),
      pub_options);
    handle1_ = negotiated_pub1_->add_upstream_negotiated_subscription(negotiated_sub_);

    negotiated_pub2_ = std::make_shared<negotiated::NegotiatedPublisher>(
      *this,
      "downstream/example2");
    negotiated_pub2_->add_supported_type<negotiated_examples::StringT2>(
      0.5,
      rclcpp::QoS(1),
      pub_options);
    handle2_ = negotiated_pub2_->add_upstream_negotiated_subscription(negotiated_sub_);

    negotiated_sub_->start();

    negotiated_pub1_->start();

    negotiated_pub2_->start();
  }

  ~NegotiatedPubSubPairExample2()
  {
    // This isn't strictly necessary, as the component is going down.  However, it drops an
    // additional reference to the negotiated_sub_, which should allow it to be properly cleaned
    // up (and reported as so in dynamic memory tracers, like asan).
    negotiated_pub1_->remove_upstream_negotiated_subscription(handle1_.get());
    negotiated_pub2_->remove_upstream_negotiated_subscription(handle2_.get());
  }

private:
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub1_;
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub2_;
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;
  std::shared_ptr<negotiated::NegotiatedPublisher::UpstreamNegotiatedSubscriptionHandle> handle1_;
  std::shared_ptr<negotiated::NegotiatedPublisher::UpstreamNegotiatedSubscriptionHandle> handle2_;
};

}  // namespace negotiated_examples

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegotiatedPubSubPairExample2)
