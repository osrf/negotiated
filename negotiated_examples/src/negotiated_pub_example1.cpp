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

#include "example_type_info.hpp"

namespace negotiated_examples
{

class NegotiatedPubExample1 final : public rclcpp::Node
{
public:
  explicit NegotiatedPubExample1(const rclcpp::NodeOptions & options)
  : rclcpp::Node("negotiated_pub_example1", options)
  {
    bool use_intra_process = this->declare_parameter("use_intra_process", false);

    double string_a_weight = this->declare_parameter("string_a_weight", 0.2);
    double int32_weight = this->declare_parameter("int32_weight", 1.0);
    double string_b_weight = this->declare_parameter("string_b_weight", 0.1);

    negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(*this, "example");
    rclcpp::PublisherOptions pub_options;
    if (use_intra_process) {
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    negotiated_pub_->add_supported_type<negotiated_examples::StringT>(
      string_a_weight,
      rclcpp::QoS(1),
      pub_options);
    negotiated_pub_->add_supported_type<negotiated_examples::Int32T>(
      int32_weight,
      rclcpp::QoS(1),
      pub_options);
    negotiated_pub_->add_supported_type<negotiated_examples::StringT2>(
      string_b_weight,
      rclcpp::QoS(1),
      pub_options);
    negotiated_pub_->start();

    auto publish_message = [this]() -> void
      {
        if (negotiated_pub_->type_was_negotiated<negotiated_examples::StringT>()) {
          auto msg = std_msgs::msg::String();
          msg.data = "Hello World: " + std::to_string(count_);
          negotiated_pub_->publish<negotiated_examples::StringT>(msg);
        }

        if (negotiated_pub_->type_was_negotiated<negotiated_examples::Int32T>()) {
          auto msg = std_msgs::msg::Int32();
          msg.data = count_;
          negotiated_pub_->publish<negotiated_examples::Int32T>(msg);
        }

        if (negotiated_pub_->type_was_negotiated<negotiated_examples::StringT2>()) {
          auto msg = std::make_unique<std_msgs::msg::String>();
          msg->data = "Hello Universe: " + std::to_string(count_);
          negotiated_pub_->publish<negotiated_examples::StringT2>(std::move(msg));
        }

        count_++;
      };

    timer_ = create_wall_timer(std::chrono::seconds(1), publish_message);
  }

private:
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_{0};
};

}  // namespace negotiated_examples

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegotiatedPubExample1)
