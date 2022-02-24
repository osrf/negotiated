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

class NegotiatedPubCompatibleExample1 final : public rclcpp::Node
{
public:
  explicit NegotiatedPubCompatibleExample1(const rclcpp::NodeOptions & options)
  : rclcpp::Node("negotiated_pub_compatible_example1", options)
  {
    compatible_pub_ = create_publisher<std_msgs::msg::String>("compatible", 10);

    negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(*this, "example");

    negotiated_pub_->add_compatible_publisher(compatible_pub_, "a", 1.0);

    negotiated_pub_->start();

    auto publish_message = [this]() -> void
      {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello World: " + std::to_string(count_);
        compatible_pub_->publish(msg);

        count_++;
      };

    timer_ = create_wall_timer(std::chrono::seconds(1), publish_message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compatible_pub_;
  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_{0};
};

}  // namespace negotiated_examples

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegotiatedPubCompatibleExample1)
