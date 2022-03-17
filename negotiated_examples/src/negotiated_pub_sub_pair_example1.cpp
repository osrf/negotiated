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

class NegotiatedPubSubPairExample1 final : public rclcpp::Node
{
public:
  explicit NegotiatedPubSubPairExample1(const rclcpp::NodeOptions & options)
  : rclcpp::Node("negotiated_pub_sub_pair_example1", options)
  {
    bool use_intra_process = this->declare_parameter("use_intra_process", false);

    double pub_string_a_weight = this->declare_parameter("pub_string_a_weight", 0.1);
    double pub_int32_weight = this->declare_parameter("pub_int32_weight", 0.5);

    double sub_string_a_weight = this->declare_parameter("sub_string_a_weight", 1.0);
    double sub_int32_weight = this->declare_parameter("sub_int32_weight", 0.5);

    auto string_user_cb = [this](std::unique_ptr<std_msgs::msg::String> msg)
      {
        RCLCPP_INFO(this->get_logger(), "String user callback: %s", msg->data.c_str());
        if (negotiated_pub_->type_was_negotiated<negotiated_examples::StringT>()) {
          msg->data += " intermediate";
          negotiated_pub_->publish<negotiated_examples::StringT>(std::move(msg));
        } else {
          // If the negotiated_pub chose Int32T, there is no way to send along the
          // data here, so just print a warning.
          RCLCPP_WARN(this->get_logger(), "Could not send String to Int32, skipping");
        }
      };

    auto int_user_cb = [this](const std_msgs::msg::Int32 & msg)
      {
        RCLCPP_INFO(this->get_logger(), "Int user callback: %d", msg.data);
        if (negotiated_pub_->type_was_negotiated<negotiated_examples::Int32T>()) {
          std_msgs::msg::Int32 int_msg;
          int_msg.data = msg.data * 2;
          negotiated_pub_->publish<negotiated_examples::Int32T>(int_msg);
        } else if (negotiated_pub_->type_was_negotiated<negotiated_examples::StringT>()) {
          std_msgs::msg::String str_msg;
          str_msg.data = std::to_string(msg.data * 2);
          negotiated_pub_->publish<negotiated_examples::StringT>(str_msg);
        }
      };

    negotiated_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(*this, "example");

    rclcpp::SubscriptionOptions sub_options;
    if (use_intra_process) {
      sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }
    if (sub_string_a_weight >= 0.0) {
      negotiated_sub_->add_supported_callback<negotiated_examples::StringT>(
        sub_string_a_weight,
        rclcpp::QoS(1),
        string_user_cb,
        sub_options);
    }
    if (sub_int32_weight >= 0.0) {
      negotiated_sub_->add_supported_callback<negotiated_examples::Int32T>(
        sub_int32_weight,
        rclcpp::QoS(1),
        int_user_cb,
        sub_options);
    }

    negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(
      *this,
      "downstream/example");

    rclcpp::PublisherOptions pub_options;
    if (use_intra_process) {
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    if (pub_string_a_weight >= 0.0) {
      negotiated_pub_->add_supported_type<negotiated_examples::StringT>(
        pub_string_a_weight,
        rclcpp::QoS(1),
        pub_options);
    }
    if (pub_int32_weight >= 0.0) {
      negotiated_pub_->add_supported_type<negotiated_examples::Int32T>(
        pub_int32_weight,
        rclcpp::QoS(1),
        pub_options);
    }

    handle_ = negotiated_pub_->add_upstream_negotiated_subscription(negotiated_sub_);

    negotiated_sub_->start();

    negotiated_pub_->start();
  }

  ~NegotiatedPubSubPairExample1()
  {
    // This isn't strictly necessary, as the component is going down.  However, it drops an
    // additional reference to the negotiated_sub_, which should allow it to be properly cleaned
    // up (and reported as so in dynamic memory tracers, like asan).
    negotiated_pub_->remove_upstream_negotiated_subscription(handle_.get());
  }

private:
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;
  std::shared_ptr<negotiated::NegotiatedSubscription> negotiated_sub_;
  std::shared_ptr<negotiated::detail::UpstreamNegotiatedSubscriptionHandle> handle_;
};

}  // namespace negotiated_examples

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegotiatedPubSubPairExample1)
