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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "negotiated/negotiated_publisher.hpp"

class TestNegotiatedPublisher : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_negotiated_publisher");
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

TEST_F(TestNegotiatedPublisher, node_constructor)
{
  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");
  ASSERT_NE(pub, nullptr);
}

TEST_F(TestNegotiatedPublisher, node_base_constructor)
{
  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(
    node_->get_node_parameters_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_base_interface(),
    node_->get_node_timers_interface(),
    "foo");
  ASSERT_NE(pub, nullptr);
}

TEST_F(TestNegotiatedPublisher, add_duplicate_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  pub.add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  EXPECT_THROW(pub.add_supported_type<EmptyT>(1.0, rclcpp::QoS(10)), std::runtime_error);
}

TEST_F(TestNegotiatedPublisher, add_callback_empty_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(pub.add_supported_type<InvalidT>(1.0, rclcpp::QoS(10)), std::runtime_error);
}

TEST_F(TestNegotiatedPublisher, remove_callback_empty_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(pub.remove_supported_type<InvalidT>(), std::runtime_error);
}

TEST_F(TestNegotiatedPublisher, remove_nonexistent_callback)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(pub.remove_supported_type<EmptyT>(), std::runtime_error);
}

TEST_F(TestNegotiatedPublisher, type_not_yet_negotiated)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  ASSERT_FALSE(pub.type_was_negotiated<EmptyT>());
}

TEST_F(TestNegotiatedPublisher, single_subscription_negotiated)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "a";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());

  std_msgs::msg::Empty empty;
  pub->publish<EmptyT>(empty);

  auto dummy_data_cb = [&empty_count]() -> bool
    {
      return empty_count > 0;
    };
  spin_while_waiting(dummy_data_cb);
  ASSERT_EQ(empty_count, 1);
}

TEST_F(TestNegotiatedPublisher, single_failed_negotiation)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "b";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_FALSE(pub->type_was_negotiated<EmptyT>());
}

TEST_F(TestNegotiatedPublisher, two_subscriptions_same_type)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "a";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  auto dummy_sub_types2 = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types2;
  negotiated_interfaces::msg::SupportedType empty_type2;
  empty_type2.ros_type_name = "std_msgs/msg/Empty";
  empty_type2.supported_type_name = "a";
  empty_type2.weight = 1.0;
  dummy_supported_types2.supported_types.push_back(empty_type2);

  dummy_sub_types2->publish(dummy_supported_types2);

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());

  std_msgs::msg::Empty empty;
  pub->publish<EmptyT>(empty);

  auto dummy_data_cb = [&empty_count]() -> bool
    {
      return empty_count > 1;
    };
  spin_while_waiting(dummy_data_cb);
  ASSERT_EQ(empty_count, 2);
}

TEST_F(TestNegotiatedPublisher, two_subscriptions_different_types)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "a";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  auto dummy_sub_types2 = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types2;
  negotiated_interfaces::msg::SupportedType empty_type2;
  empty_type2.ros_type_name = "std_msgs/msg/String";
  empty_type2.supported_type_name = "b";
  empty_type2.weight = 1.0;
  dummy_supported_types2.supported_types.push_back(empty_type2);

  dummy_sub_types2->publish(dummy_supported_types2);

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_string_cb = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_string_cb);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());

  std_msgs::msg::Empty empty;
  pub->publish<EmptyT>(empty);

  auto str = std::make_unique<std_msgs::msg::String>();
  pub->publish<StringT>(std::move(str));

  auto dummy_data_cb = [&empty_count, &string_count]() -> bool
    {
      return empty_count > 0 && string_count > 0;
    };
  spin_while_waiting(dummy_data_cb);
  ASSERT_EQ(empty_count, 1);
  ASSERT_EQ(string_count, 1);
}

TEST_F(TestNegotiatedPublisher, disconnect_supported_types)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "a";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  auto dummy_sub_types2 = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types2;
  negotiated_interfaces::msg::SupportedType empty_type2;
  empty_type2.ros_type_name = "std_msgs/msg/String";
  empty_type2.supported_type_name = "b";
  empty_type2.weight = 1.0;
  dummy_supported_types2.supported_types.push_back(empty_type2);

  dummy_sub_types2->publish(dummy_supported_types2);

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_string_cb = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_string_cb);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());

  // Disconnect one of the supported_types
  dummy_sub_types2.reset();

  auto negotiated_break_cb2 = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && !pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb2);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_FALSE(pub->type_was_negotiated<StringT>());
}

TEST_F(TestNegotiatedPublisher, dont_negotiate_on_subscription_add)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "a";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  auto dummy_sub_types2 = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types2;
  negotiated_interfaces::msg::SupportedType empty_type2;
  empty_type2.ros_type_name = "std_msgs/msg/String";
  empty_type2.supported_type_name = "b";
  empty_type2.weight = 1.0;
  dummy_supported_types2.supported_types.push_back(empty_type2);

  dummy_sub_types2->publish(dummy_supported_types2);

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_string_cb = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_string_cb);

  negotiated::NegotiatedPublisherOptions options;
  options.negotiate_on_subscription_add = false;

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo", options);

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_FALSE(pub->type_was_negotiated<EmptyT>());
  ASSERT_FALSE(pub->type_was_negotiated<StringT>());

  pub->negotiate();

  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());
}

TEST_F(TestNegotiatedPublisher, dont_negotiate_on_disconnect)
{
  // Dummy subscription
  auto dummy_sub_types = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types;
  negotiated_interfaces::msg::SupportedType empty_type;
  empty_type.ros_type_name = "std_msgs/msg/Empty";
  empty_type.supported_type_name = "a";
  empty_type.weight = 1.0;
  dummy_supported_types.supported_types.push_back(empty_type);

  dummy_sub_types->publish(dummy_supported_types);

  auto dummy_sub_types2 = node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
    "foo/supported_types", rclcpp::QoS(10).transient_local());

  negotiated_interfaces::msg::SupportedTypes dummy_supported_types2;
  negotiated_interfaces::msg::SupportedType empty_type2;
  empty_type2.ros_type_name = "std_msgs/msg/String";
  empty_type2.supported_type_name = "b";
  empty_type2.weight = 1.0;
  dummy_supported_types2.supported_types.push_back(empty_type2);

  dummy_sub_types2->publish(dummy_supported_types2);

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_string_cb = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_string_cb);

  negotiated::NegotiatedPublisherOptions options;
  options.negotiate_on_subscription_removal = false;

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(1.0, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());

  // Disconnect one of the supported_types
  dummy_sub_types2.reset();

  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());

  pub->negotiate();

  auto negotiated_break_cb2 = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && !pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb2);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_FALSE(pub->type_was_negotiated<StringT>());
}
