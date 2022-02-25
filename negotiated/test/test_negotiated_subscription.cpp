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
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

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

    topics_pub_ = node_->create_publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>(
      "foo", rclcpp::QoS(10));
  }

  void TearDown()
  {
    node_.reset();
  }

  bool spin_while_waiting(std::function<bool()> break_func)
  {
    // Setup our bogus future so we can spin
    std::promise<bool> promise;
    std::future<bool> future = promise.get_future();
    auto shared_future = future.share();

    for (int i = 0; i < 20; ++i) {
      if (break_func()) {
        return true;
      }
      rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
    }

    return false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr topics_pub_;
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

struct InvalidT
{
  using MsgT = std_msgs::msg::Empty;
  static const inline std::string supported_type_name = "";
};

TEST_F(TestNegotiatedSubscription, node_constructor)
{
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");
  ASSERT_NE(sub, nullptr);
}

TEST_F(TestNegotiatedSubscription, node_base_constructor)
{
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    node_->get_node_parameters_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_logging_interface(),
    "foo");
  ASSERT_NE(sub, nullptr);
}

TEST_F(TestNegotiatedSubscription, add_callback_empty_type)
{
  negotiated::NegotiatedSubscription sub(*node_, "foo");

  auto cb = [](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
    };

  EXPECT_THROW(
    sub.add_supported_callback<InvalidT>(1.0, rclcpp::QoS(10), cb),
    std::invalid_argument);
}

TEST_F(TestNegotiatedSubscription, add_duplicate_callback)
{
  negotiated::NegotiatedSubscription sub(*node_, "foo");

  auto cb = [](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
    };

  sub.add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), cb);
  EXPECT_THROW(sub.add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), cb), std::invalid_argument);
}

TEST_F(TestNegotiatedSubscription, remove_callback_empty_type)
{
  negotiated::NegotiatedSubscription sub(*node_, "foo");

  EXPECT_THROW(sub.remove_supported_callback<InvalidT>(), std::invalid_argument);
}

TEST_F(TestNegotiatedSubscription, remove_nonexistent_callback)
{
  negotiated::NegotiatedSubscription sub(*node_, "foo");

  EXPECT_THROW(sub.remove_supported_callback<EmptyT>(), std::invalid_argument);
}

TEST_F(TestNegotiatedSubscription, add_single_callback)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::Empty>("foo/a", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/Empty";
  topic_info.supported_type_name = "a";
  topic_info.topic_name = "foo/a";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

  int count = 0;
  auto empty_cb = [&count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      count++;
    };

  sub->add_supported_callback<EmptyT>(1.0, rclcpp::QoS(10), empty_cb);
  sub->start();

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(std::move(topics_msg));

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::Empty data;
  data_pub->publish(data);

  auto count_break_func = [&count]() -> bool
    {
      return count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(count, 1);
}

TEST_F(TestNegotiatedSubscription, add_multiple_callbacks)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

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

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(std::move(topics_msg));

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::String data;
  data_pub->publish(data);

  auto count_break_func = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);
}

TEST_F(TestNegotiatedSubscription, failed_topics_info)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = false;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

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

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(std::move(topics_msg));

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 &&
             data_pub->get_subscription_count() > 0;
    };
  ASSERT_FALSE(spin_while_waiting(data_break_func));
}

TEST_F(TestNegotiatedSubscription, no_matching_topics)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/c", rclcpp::QoS(10));

  auto topics_msg = std::make_unique<negotiated_interfaces::msg::NegotiatedTopicsInfo>();
  topics_msg->success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "c";
  topic_info.topic_name = "foo/c";
  topics_msg->negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

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

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(std::move(topics_msg));

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 &&
             data_pub->get_subscription_count() > 0;
    };
  ASSERT_FALSE(spin_while_waiting(data_break_func));
}

TEST_F(TestNegotiatedSubscription, renegotiate_keep_same_topic)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  negotiated_interfaces::msg::NegotiatedTopicsInfo topics_msg;
  topics_msg.success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg.negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

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

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(topics_msg);

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::String data;
  data_pub->publish(data);

  auto count_break_func = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);

  // Now renegotiate
  empty_count = 0;
  string_count = 0;

  topics_msg.success = true;
  topics_msg.negotiated_topics.clear();

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info1;
  topic_info1.ros_type_name = "std_msgs/msg/Empty";
  topic_info1.supported_type_name = "a";
  topic_info1.topic_name = "foo/a";
  topics_msg.negotiated_topics.push_back(topic_info1);

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info2;
  topic_info2.ros_type_name = "std_msgs/msg/String";
  topic_info2.supported_type_name = "b";
  topic_info2.topic_name = "foo/b";
  topics_msg.negotiated_topics.push_back(topic_info2);

  topics_pub_->publish(topics_msg);

  ASSERT_TRUE(spin_while_waiting(data_break_func));

  data_pub->publish(data);

  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);
}

TEST_F(TestNegotiatedSubscription, renegotiate_change_topic)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto data_pub2 = node_->create_publisher<std_msgs::msg::Empty>("foo/a", rclcpp::QoS(10));

  negotiated_interfaces::msg::NegotiatedTopicsInfo topics_msg;
  topics_msg.success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg.negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

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

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(topics_msg);

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::String data;
  data_pub->publish(data);

  auto count_break_func = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);

  // Now renegotiate
  empty_count = 0;
  string_count = 0;

  topics_msg.success = true;
  topics_msg.negotiated_topics.clear();

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info1;
  topic_info1.ros_type_name = "std_msgs/msg/Empty";
  topic_info1.supported_type_name = "a";
  topic_info1.topic_name = "foo/a";
  topics_msg.negotiated_topics.push_back(topic_info1);

  topics_pub_->publish(topics_msg);

  auto data2_break_func = [sub, data_pub2]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub2->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data2_break_func));

  std_msgs::msg::Empty data2;
  data_pub2->publish(data2);

  auto count2_break_func = [&empty_count]() -> bool
    {
      return empty_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count2_break_func));

  ASSERT_EQ(empty_count, 1);
  ASSERT_EQ(string_count, 0);
}

TEST_F(TestNegotiatedSubscription, custom_negotiated_cb)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  negotiated_interfaces::msg::NegotiatedTopicsInfo topics_msg;
  topics_msg.success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info1;
  topic_info1.ros_type_name = "std_msgs/msg/Empty";
  topic_info1.supported_type_name = "a";
  topic_info1.topic_name = "foo/a";
  topics_msg.negotiated_topics.push_back(topic_info1);

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info2;
  topic_info2.ros_type_name = "std_msgs/msg/String";
  topic_info2.supported_type_name = "b";
  topic_info2.topic_name = "foo/b";
  topics_msg.negotiated_topics.push_back(topic_info2);

  // Both the NegotiatedSubscription and the "fake" publisher express a preference for the
  // std_msgs/msg/Empty publication, so by default that is what the NegotiatedSubscription
  // would choose.  The custom negotiation function below chooses the std_msgs/msg/String
  // one instead.
  auto custom_negotiate_cb =
    [](const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
      const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg) ->
    negotiated_interfaces::msg::NegotiatedTopicInfo
    {
      (void)existing_info;

      if (msg.negotiated_topics.size() < 2) {
        return negotiated_interfaces::msg::NegotiatedTopicInfo();
      }

      return msg.negotiated_topics[1];
    };

  negotiated::NegotiatedSubscriptionOptions negotiated_sub_options;
  negotiated_sub_options.negotiate_cb = custom_negotiate_cb;

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    *node_,
    "foo",
    negotiated_sub_options);

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
  sub->add_supported_callback<StringT>(0.5, rclcpp::QoS(10), string_cb);

  sub->start();

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(topics_msg);

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::String data;
  data_pub->publish(data);

  auto count_break_func = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);
}

TEST_F(TestNegotiatedSubscription, renegotiate_after_remove_topic)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto data_pub2 = node_->create_publisher<std_msgs::msg::Empty>("foo/a", rclcpp::QoS(10));

  negotiated_interfaces::msg::NegotiatedTopicsInfo topics_msg;
  topics_msg.success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info1;
  topic_info1.ros_type_name = "std_msgs/msg/Empty";
  topic_info1.supported_type_name = "a";
  topic_info1.topic_name = "foo/a";
  topics_msg.negotiated_topics.push_back(topic_info1);

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info2;
  topic_info2.ros_type_name = "std_msgs/msg/String";
  topic_info2.supported_type_name = "b";
  topic_info2.topic_name = "foo/b";
  topics_msg.negotiated_topics.push_back(topic_info2);

  // Setup and test the subscription
  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(*node_, "foo");

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
  sub->add_supported_callback<StringT>(0.5, rclcpp::QoS(10), string_cb);

  sub->start();

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(topics_msg);

  auto data_break_func = [sub, data_pub2]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub2->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::Empty data;
  data_pub2->publish(data);

  auto count_break_func = [&empty_count]() -> bool
    {
      return empty_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 1);
  ASSERT_EQ(string_count, 0);

  // Now remove the NegotiatedSubscription Empty topic
  sub->remove_supported_callback<EmptyT>();

  // Now renegotiate; the subscription should change to String
  empty_count = 0;
  string_count = 0;

  topics_pub_->publish(topics_msg);

  auto data_break_func2 = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func2));

  std_msgs::msg::String data2;
  data_pub->publish(data2);

  auto count_break_func2 = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func2));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);
}

TEST_F(TestNegotiatedSubscription, fail_renegotiate)
{
  // Setup of our dummy publisher
  auto data_pub = node_->create_publisher<std_msgs::msg::String>("foo/b", rclcpp::QoS(10));

  auto data_pub2 = node_->create_publisher<std_msgs::msg::Empty>("foo/a", rclcpp::QoS(10));

  negotiated_interfaces::msg::NegotiatedTopicsInfo topics_msg;
  topics_msg.success = true;

  negotiated_interfaces::msg::NegotiatedTopicInfo topic_info;
  topic_info.ros_type_name = "std_msgs/msg/String";
  topic_info.supported_type_name = "b";
  topic_info.topic_name = "foo/b";
  topics_msg.negotiated_topics.push_back(topic_info);

  // Setup and test the subscription
  negotiated::NegotiatedSubscriptionOptions negotiated_sub_options;
  negotiated_sub_options.disconnect_on_negotiation_failure = false;

  auto sub = std::make_shared<negotiated::NegotiatedSubscription>(
    *node_,
    "foo",
    negotiated_sub_options);

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

  auto negotiated_break_func = [this, sub]() -> bool
    {
      return sub->get_negotiated_topic_publisher_count() > 0 &&
             topics_pub_->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(negotiated_break_func));

  topics_pub_->publish(topics_msg);

  auto data_break_func = [sub, data_pub]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0 && data_pub->get_subscription_count() > 0;
    };
  ASSERT_TRUE(spin_while_waiting(data_break_func));

  std_msgs::msg::String data;
  data_pub->publish(data);

  auto count_break_func = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  ASSERT_TRUE(spin_while_waiting(count_break_func));

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 1);

  // Now remove the only supported type, and cause renegotiation
  sub->remove_supported_callback<StringT>();

  topics_pub_->publish(topics_msg);

  auto data2_break_func = [sub, data_pub2]() -> bool
    {
      return sub->get_data_topic_publisher_count() > 0;
    };
  ASSERT_FALSE(spin_while_waiting(data2_break_func));
}
