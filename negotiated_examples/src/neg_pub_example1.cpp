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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_publisher.hpp"

#include "example_type_info.hpp"

namespace negotiated_examples
{

class NegPubExample1 final : public rclcpp::Node
{
public:
  explicit NegPubExample1(const rclcpp::NodeOptions & options)
  : rclcpp::Node("neg_pub_example1", options)
  {
    neg_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(this, "myneg");

    bool use_intra_process = this->declare_parameter("use_intra_process", false);
    rclcpp::PublisherOptions pub_options;
    if (use_intra_process) {
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    neg_pub_->add_supported_type<negotiated_examples::StringT>(1.0, rclcpp::QoS(1), pub_options);
    neg_pub_->add_supported_type<negotiated_examples::Int32T>(0.5, rclcpp::QoS(1), pub_options);
    neg_pub_->add_supported_type<negotiated_examples::StringT2>(0.1, rclcpp::QoS(1), pub_options);
    neg_pub_->start();

    auto publish_message = [this]() -> void
      {
        if (neg_pub_->type_was_negotiated<negotiated_examples::StringT>()) {
          auto msg = std_msgs::msg::String();
          msg.data = "Hello World: " + std::to_string(count_);
          neg_pub_->publish<negotiated_examples::StringT>(msg);
        }

        if (neg_pub_->type_was_negotiated<negotiated_examples::Int32T>()) {
          auto msg = std_msgs::msg::Int32();
          msg.data = count_;
          neg_pub_->publish<negotiated_examples::Int32T>(msg);
        }

        if (neg_pub_->type_was_negotiated<negotiated_examples::StringT2>()) {
          auto msg = std::make_unique<std_msgs::msg::String>();
          msg->data = "Hello Universe: " + std::to_string(count_);
          neg_pub_->publish<negotiated_examples::StringT2>(std::move(msg));
        }

        count_++;
      };

    timer_ = create_wall_timer(std::chrono::seconds(1), publish_message);
  }

private:
  std::shared_ptr<negotiated::NegotiatedPublisher> neg_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_{0};
};

}  // namespace negotiated_examples

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegPubExample1)
