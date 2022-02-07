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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated_interfaces/msg/negotiated_topic_info.hpp"
#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"

#include "negotiated/negotiated_subscription.hpp"

class TestNegotiatedSubscription : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

protected:
  void SetUp()
  {
    node_ = std::make_shared<rclcpp::Node>("test_negotiated_subscription");
  }

  void TearDown()
  {
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
};

struct EmptyT
{
  using MsgT = std_msgs::msg::Empty;
  static const inline std::string supported_type_name = "a";
};

struct StringT
{
  using MsgT = std_msgs::msg::String;
  static const inline std::string supported_type_name = "b";
};

TEST_F(TestNegotiatedSubscription, node_constructor)
{
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");
  ASSERT_NE(sub, nullptr);
}

TEST_F(TestNegotiatedSubscription, node_base_constructor)
{
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "foo");
  ASSERT_NE(sub, nullptr);
}

TEST_F(TestNegotiatedSubscription, add_duplicate_callback)
{
  negotiated::NegotiatedSubscription sub(
    node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "foo");

  auto cb = [](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
    };

  sub.add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), cb);
  EXPECT_THROW(sub.add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), cb), std::runtime_error);
}

TEST_F(TestNegotiatedSubscription, add_single_callback)
{
  // Setup of our dummy publisher
  auto topics_pub = node_->create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    "foo", rclcpp::QoS(10));

  auto data_pub = node_->create_publisher<std_msgs::msg::Empty>("foo/a", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/Empty";
  topic_info.supported_type_name = "a";
  topic_info.topic_name = "foo/a";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup our bogus future so we can spin
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  auto shared_future = future.share();

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "foo");

  int count = 0;
  auto empty_cb = [&count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      count++;
    };

  sub->add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), empty_cb);
  sub->start();

  bool matched = false;
  for (int i = 0; i < 200; ++i) {
    matched = sub->get_negotiated_topic_publisher_count() > 0 &&
      topics_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(matched);

  topics_pub->publish(std::move(topics_msg));

  matched = false;
  for (int i = 0; i < 200; ++i) {
    matched = sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(matched);

  std_msgs::msg::Empty data;
  data_pub->publish(data);

  for (int i = 0; i < 200; ++i) {
    if (count > 0) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }

  ASSERT_EQ(count, 1);
}

TEST_F(TestNegotiatedSubscription, add_multiple_callbacks)
{
  // Setup of our dummy publisher
  auto topics_pub = node_->create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    "foo", rclcpp::QoS(10));

  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup our bogus future so we can spin
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  auto shared_future = future.share();

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "foo");

  int empty_count = 0;
  auto empty_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto string_cb = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  sub->add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), empty_cb);
  sub->add_supported_callback<StringT>(1.0, rclcpp::QoS(10), string_cb);

  sub->start();

  bool matched = false;
  for (int i = 0; i < 200; ++i) {
    matched = sub->get_negotiated_topic_publisher_count() > 0 &&
      topics_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(matched);

  topics_pub->publish(std::move(topics_msg));

  matched = false;
  for (int i = 0; i < 200; ++i) {
    matched = sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(matched);

  std_msgs::msg::String data;
  data_pub->publish(data);

  for (int i = 0; i < 200; ++i) {
    if (string_count > 0) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);
}

TEST_F(TestNegotiatedSubscription, failed_topics_info)
{
  // Setup of our dummy publisher
  auto topics_pub = node_->create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    "foo", rclcpp::QoS(10));

  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = false;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup our bogus future so we can spin
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  auto shared_future = future.share();

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "foo");

  auto empty_cb = [](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
    };

  auto string_cb = [](const std_msgs::msg::String & msg)
    {
      (void)msg;
    };

  sub->add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), empty_cb);
  sub->add_supported_callback<StringT>(1.0, rclcpp::QoS(10), string_cb);

  sub->start();

  bool matched = false;
  for (int i = 0; i < 200; ++i) {
    matched = sub->get_negotiated_topic_publisher_count() > 0 &&
      topics_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(matched);

  topics_pub->publish(std::move(topics_msg));

  matched = false;
  for (int i = 0; i < 20; ++i) {
    matched = sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(matched);
}

TEST_F(TestNegotiatedSubscription, no_matching_topics)
{
  // Setup of our dummy publisher
  auto topics_pub = node_->create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
    "foo", rclcpp::QoS(10));

  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/c", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "c";
  topic_info.topic_name = "foo/c";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup our bogus future so we can spin
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  auto shared_future = future.share();

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node_->get_node_parameters_interface(), node_->get_node_topics_interface(), "foo");

  auto empty_cb = [](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
    };

  auto string_cb = [](const std_msgs::msg::String & msg)
    {
      (void)msg;
    };

  sub->add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), empty_cb);
  sub->add_supported_callback<StringT>(1.0, rclcpp::QoS(10), string_cb);

  sub->start();

  bool matched = false;
  for (int i = 0; i < 200; ++i) {
    matched = sub->get_negotiated_topic_publisher_count() > 0 &&
      topics_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(matched);

  topics_pub->publish(std::move(topics_msg));

  matched = false;
  for (int i = 0; i < 20; ++i) {
    matched = sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    if (matched) {
      break;
    }
    rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(matched);
}
