// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_publisher.hpp"

#include "example_type_info.hpp"

class NegotiatedLifecyclePub : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NegotiatedLifecyclePub(
    const std::string & node_name,
    bool use_intra_process_comm = false)
  : rclcpp_lifecycle::LifecycleNode(node_name),
    use_intra_process_comm_(use_intra_process_comm),
    count_(0)
  {}

  void
  publish()
  {
    if (negotiated_pub_->type_was_negotiated<negotiated_examples::StringT>()) {
      auto msg = std_msgs::msg::String();
      msg.data = "Lifecycle Hello World: " + std::to_string(count_);
      negotiated_pub_->publish<negotiated_examples::StringT>(msg);
    }

    if (negotiated_pub_->type_was_negotiated<negotiated_examples::Int32T>()) {
      auto msg = std_msgs::msg::Int32();
      msg.data = count_;
      negotiated_pub_->publish<negotiated_examples::Int32T>(msg);
    }

    if (negotiated_pub_->type_was_negotiated<negotiated_examples::StringT2>()) {
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = "Lifecycle Hello Universe: " + std::to_string(count_);
      negotiated_pub_->publish<negotiated_examples::StringT2>(std::move(msg));
    }

    count_++;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on_configure() is called.");

    rclcpp::PublisherOptions pub_options;
    if (use_intra_process_comm_) {
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    }

    negotiated_pub_ = std::make_shared<negotiated::NegotiatedPublisher>(
      get_node_parameters_interface(),
      get_node_topics_interface(),
      get_node_logging_interface(),
      get_node_graph_interface(),
      get_node_base_interface(),
      get_node_timers_interface(),
      "example");
    negotiated_pub_->add_supported_type<negotiated_examples::StringT>(
      1.0,
      rclcpp::QoS(1),
      pub_options);
    negotiated_pub_->add_supported_type<negotiated_examples::Int32T>(
      0.5,
      rclcpp::QoS(1),
      pub_options);
    negotiated_pub_->add_supported_type<negotiated_examples::StringT2>(
      0.1,
      rclcpp::QoS(1),
      pub_options);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&NegotiatedLifecyclePub::publish, this));
    // TODO(clalancette): This is racy because the timer may actually fire before we
    // cancel it.  This is a weakness of the current rclcpp::TimerBase API.
    timer_->cancel();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "on_activate() is called.");

    LifecycleNode::on_activate(state);

    negotiated_pub_->start();

    timer_->reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

    LifecycleNode::on_deactivate(state);

    timer_->cancel();

    negotiated_pub_->stop();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "on cleanup is called.");

    timer_.reset();
    negotiated_pub_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(
      get_logger(),
      "on shutdown is called from state %s.",
      state.label().c_str());

    timer_.reset();
    negotiated_pub_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  bool use_intra_process_comm_;
  int count_;

  std::shared_ptr<negotiated::NegotiatedPublisher> negotiated_pub_;

  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto negotiated_lifecycle_node = std::make_shared<NegotiatedLifecyclePub>(
    "negotiated_lifecycle_pub");

  executor.add_node(negotiated_lifecycle_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
