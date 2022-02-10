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

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_publisher.hpp"

template<>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = std_msgs::msg::String;

  static
  void
  convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination.data = source;
  }

  static
  void
  convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = source.data;
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  std::string,
  std_msgs::msg::String);

struct StringT
{
  // This could also be:
  // using MsgT = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;
  using MsgT = std::string;
  static const inline std::string supported_type_name = "a";
};

namespace negotiated_examples
{

class NegotiatedPubTypeAdaptExample1 final : public rclcpp::Node
{
public:
  explicit NegotiatedPubTypeAdaptExample1(const rclcpp::NodeOptions & options)
  : rclcpp::Node("negotiated_pub_type_adapt_example1", options)
  {
    negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(*this, "example");

    bool use_intra_process = this->declare_parameter("use_intra_process", false);
    rclcpp::PublisherOptions pub_options;
    if (use_intra_process) {
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    negotiated_pub_->add_supported_type<StringT>(
      1.0,
      rclcpp::QoS(1),
      pub_options);
    negotiated_pub_->start();

    auto publish_message = [this]() -> void
      {
        if (negotiated_pub_->type_was_negotiated<StringT>()) {
          std::string msg = "Hello World: " + std::to_string(count_);
          negotiated_pub_->publish<StringT>(msg);
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

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegotiatedPubTypeAdaptExample1)
