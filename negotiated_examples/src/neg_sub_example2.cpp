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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_subscription.hpp"

#include "example_type_info.hpp"

namespace negotiated_examples
{

class NegSubExample2 final : public rclcpp::Node
{
public:
  explicit NegSubExample2(const rclcpp::NodeOptions & options)
  : rclcpp::Node("neg_sub_example2", options)
  {
    auto string_user_cb = [this](const std_msgs::msg::String & msg)
      {
        RCLCPP_INFO(get_logger(), "String user callback: %s", msg.data.c_str());
      };

    neg_sub_ = std::make_shared<negotiated::NegotiatedSubscription>(
      this->get_node_parameters_interface(),
      this->get_node_topics_interface(),
      "myneg");
    neg_sub_->add_supported_callback<negotiated_examples::StringT2>(
      1.0,
      rclcpp::QoS(1),
      string_user_cb);
    neg_sub_->start();
  }

private:
  std::shared_ptr<negotiated::NegotiatedSubscription> neg_sub_;
};

}  // namespace negotiated_examples

RCLCPP_COMPONENTS_REGISTER_NODE(negotiated_examples::NegSubExample2)
