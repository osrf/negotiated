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

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

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

    for (int i = 0; i < 50; ++i) {
      if (break_func()) {
        return true;
      }
      rclcpp::spin_until_future_complete(node_, shared_future, std::chrono::milliseconds(10));
    }

    return false;
  }

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr
  create_and_publish_foo_supported_types(
    const std::vector<std::string> & ros_type_names,
    const std::vector<std::string> & supported_type_names)
  {
    if (ros_type_names.size() != supported_type_names.size()) {
      throw std::invalid_argument(
              "ros_type_names and supported_type_names must be the same length");
    }

    auto dummy_foo_supported_types =
      node_->create_publisher<negotiated_interfaces::msg::SupportedTypes>(
      "foo/supported_types", rclcpp::QoS(10).transient_local());

    negotiated_interfaces::msg::SupportedTypes dummy_supported_types;

    for (size_t i = 0; i < ros_type_names.size(); ++i) {
      negotiated_interfaces::msg::SupportedType supported_type;
      supported_type.ros_type_name = ros_type_names[i];
      supported_type.supported_type_name = supported_type_names[i];
      supported_type.weight = 1.0;
      dummy_supported_types.supported_types.push_back(supported_type);
    }

    dummy_foo_supported_types->publish(dummy_supported_types);

    return dummy_foo_supported_types;
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

TEST_F(TestNegotiatedPublisher, zero_negotiated_solutions)
{
  negotiated::NegotiatedPublisherOptions options;
  options.maximum_negotiated_solutions = 0;

  EXPECT_THROW(
    std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo", options),
    std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, add_duplicate_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  pub.add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  EXPECT_THROW(pub.add_supported_type<EmptyT>(1.0, rclcpp::QoS(10)), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, add_callback_empty_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(pub.add_supported_type<InvalidT>(1.0, rclcpp::QoS(10)), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, remove_callback_empty_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(pub.remove_supported_type<InvalidT>(), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, remove_nonexistent_callback)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(pub.remove_supported_type<EmptyT>(), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, type_not_yet_negotiated)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  ASSERT_FALSE(pub.type_was_negotiated<EmptyT>());
}

TEST_F(TestNegotiatedPublisher, single_subscription_negotiated)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

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
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"b"});

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
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

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
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

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
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

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
  dummy2.reset();

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
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

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
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

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
  dummy2.reset();

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

TEST_F(TestNegotiatedPublisher, two_subscriptions_different_types_no_multiple_types)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

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
  options.maximum_negotiated_solutions = 1;

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

  ASSERT_EQ(empty_count, 0);
  ASSERT_EQ(string_count, 0);
}

TEST_F(TestNegotiatedPublisher, successful_negotiation_callback)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  bool negotiation_cb_called = false;

  negotiated::NegotiatedPublisherOptions options;
  options.successful_negotiation_cb =
    [&negotiation_cb_called](const negotiated_interfaces::msg::NegotiatedTopicsInfo & topics)
    {
      (void)topics;
      negotiation_cb_called = true;
    };

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo", options);

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

  // And now let's ensure that the negotiated callback was called
  ASSERT_TRUE(negotiation_cb_called);
}

TEST_F(TestNegotiatedPublisher, negotiation_callback)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types(
    {"std_msgs/msg/Empty", "std_msgs/msg/String"},
    {"a", "b"});

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_cb2 = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_cb2);

  negotiated::NegotiatedPublisherOptions options;
  options.negotiation_cb =
    [](const std::map<negotiated::detail::PublisherGid,
      std::vector<std::string>> & negotiated_sub_gid_to_keys, const std::map<std::string,
      negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
      size_t maximum_solutions) -> std::vector<negotiated_interfaces::msg::SupportedType>
    {
      (void)negotiated_sub_gid_to_keys;
      (void)key_to_supported_types;
      (void)maximum_solutions;

      // By default, we would have expected the negotiation to choose the std_msgs/msg/Empty
      // ROS message type, since that had the higher weight.  But this negotiation algorithm
      // always prefers the std_msgs/msg/String one.
      std::vector<negotiated_interfaces::msg::SupportedType> ret(1);
      ret[0].ros_type_name = "std_msgs/msg/String";
      ret[0].supported_type_name = "b";
      return ret;
    };

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo", options);

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(0.1, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());
  ASSERT_FALSE(pub->type_was_negotiated<EmptyT>());

  std_msgs::msg::String str;
  pub->publish<StringT>(str);

  auto dummy_data_cb = [&string_count]() -> bool
    {
      return string_count > 0;
    };
  spin_while_waiting(dummy_data_cb);
  ASSERT_EQ(string_count, 1);
  ASSERT_EQ(empty_count, 0);
}

TEST_F(TestNegotiatedPublisher, negotiation_callback_empty_set)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types(
    {"std_msgs/msg/Empty", "std_msgs/msg/String"},
    {"a", "b"});

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_cb2 = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_cb2);

  negotiated::NegotiatedPublisherOptions options;
  options.negotiation_cb =
    [](const std::map<negotiated::detail::PublisherGid,
      std::vector<std::string>> & negotiated_sub_gid_to_keys, const std::map<std::string,
      negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
      size_t maximum_solutions) -> std::vector<negotiated_interfaces::msg::SupportedType>
    {
      (void)negotiated_sub_gid_to_keys;
      (void)key_to_supported_types;
      (void)maximum_solutions;

      // By default, we would have expected the negotiation to choose the std_msgs/msg/Empty
      // ROS message type, but this negotiation algorithm couldn't find a solution and returns
      // an empty vector.
      return std::vector<negotiated_interfaces::msg::SupportedType>();
    };

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo", options);

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(0.1, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_FALSE(pub->type_was_negotiated<StringT>());
  ASSERT_FALSE(pub->type_was_negotiated<EmptyT>());
}

TEST_F(TestNegotiatedPublisher, negotiation_callback_bogus_data)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types(
    {"std_msgs/msg/Empty", "std_msgs/msg/String"},
    {"a", "b"});

  int empty_count = 0;
  auto dummy_sub_cb = [&empty_count](const std_msgs::msg::Empty & msg)
    {
      (void)msg;
      empty_count++;
    };

  int string_count = 0;
  auto dummy_sub_cb2 = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::Empty>(
    "foo/a", rclcpp::QoS(10), dummy_sub_cb);

  auto dummy_sub2 = node_->create_subscription<std_msgs::msg::String>(
    "foo/b", rclcpp::QoS(10), dummy_sub_cb2);

  negotiated::NegotiatedPublisherOptions options;
  options.negotiation_cb =
    [](const std::map<negotiated::detail::PublisherGid,
      std::vector<std::string>> & negotiated_sub_gid_to_keys, const std::map<std::string,
      negotiated::detail::SupportedTypeInfo> & key_to_supported_types,
      size_t maximum_solutions) -> std::vector<negotiated_interfaces::msg::SupportedType>
    {
      (void)negotiated_sub_gid_to_keys;
      (void)key_to_supported_types;
      (void)maximum_solutions;

      // This negotiation algorithm somehow chose an invalid combination; the rest of the
      // code in NegotiatedPublisher should reject this.
      std::vector<negotiated_interfaces::msg::SupportedType> ret(1);
      ret[0].ros_type_name = "std_msgs/msg/Bogus";
      ret[0].supported_type_name = "q";
      return ret;
    };

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo", options);

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_supported_type<StringT>(0.1, rclcpp::QoS(10));

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_FALSE(pub->type_was_negotiated<StringT>());
  ASSERT_FALSE(pub->type_was_negotiated<EmptyT>());
}

TEST_F(TestNegotiatedPublisher, add_compatible_publisher_nullptr)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  EXPECT_THROW(
    pub.add_compatible_publisher<std_msgs::msg::String>(
      nullptr, "blah", 1.0), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, add_compatible_publisher_empty_supported_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  EXPECT_THROW(pub.add_compatible_publisher(compat_pub, "", 1.0), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, add_compatible_publisher_duplicate_with_compat)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  pub.add_compatible_publisher(compat_pub, "a", 1.0);
  EXPECT_THROW(pub.add_compatible_publisher(compat_pub, "a", 1.0), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, add_compatible_publisher_duplicate_with_negotiated)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  pub.add_supported_type<StringT>(1.0, rclcpp::QoS(10));

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  EXPECT_THROW(pub.add_compatible_publisher(compat_pub, "b", 1.0), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, remove_compatible_publisher_empty_supported_type)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  EXPECT_THROW(pub.remove_compatible_publisher(compat_pub, ""), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, remove_compatible_publisher_non_existent)
{
  negotiated::NegotiatedPublisher pub(*node_, "foo");

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  EXPECT_THROW(pub.remove_compatible_publisher(compat_pub, "a"), std::invalid_argument);
}

TEST_F(TestNegotiatedPublisher, single_compatible_subscription_negotiated)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

  int string_count = 0;
  auto dummy_sub_cb = [&string_count](const std_msgs::msg::String & msg)
    {
      (void)msg;
      string_count++;
    };

  auto dummy_sub = node_->create_subscription<std_msgs::msg::String>(
    "compat", rclcpp::QoS(10), dummy_sub_cb);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  pub->add_compatible_publisher(compat_pub, "b", 1.0);

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());

  // Publish using the compat_pub
  std_msgs::msg::String str;
  compat_pub->publish(str);

  // Publish using the negotiated publisher pub
  pub->publish<StringT>(str);

  auto dummy_data_cb = [&string_count]() -> bool
    {
      return string_count > 1;
    };
  spin_while_waiting(dummy_data_cb);
  ASSERT_EQ(string_count, 2);
}

TEST_F(TestNegotiatedPublisher, one_compat_one_negotiated_sub)
{
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy =
    create_and_publish_foo_supported_types({"std_msgs/msg/Empty"}, {"a"});

  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr dummy2 =
    create_and_publish_foo_supported_types({"std_msgs/msg/String"}, {"b"});

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

  auto dummy_sub_string = node_->create_subscription<std_msgs::msg::String>(
    "compat", rclcpp::QoS(10), dummy_sub_string_cb);

  auto pub = std::make_shared<negotiated::NegotiatedPublisher>(*node_, "foo");

  auto compat_pub = node_->create_publisher<std_msgs::msg::String>("compat", 10);

  pub->add_supported_type<EmptyT>(1.0, rclcpp::QoS(10));
  pub->add_compatible_publisher(compat_pub, "b", 1.0);

  pub->start();

  auto negotiated_break_cb = [pub]() -> bool
    {
      return pub->type_was_negotiated<EmptyT>() && pub->type_was_negotiated<StringT>();
    };
  spin_while_waiting(negotiated_break_cb);
  ASSERT_TRUE(pub->type_was_negotiated<EmptyT>());
  ASSERT_TRUE(pub->type_was_negotiated<StringT>());

  // Publish using the compat_pub
  std_msgs::msg::String str;
  compat_pub->publish(str);

  // Publish using the negotiated publisher pub
  pub->publish<StringT>(str);

  std_msgs::msg::Empty empty;
  pub->publish<EmptyT>(empty);

  auto dummy_data_cb = [&string_count, &empty_count]() -> bool
    {
      return string_count > 1 && empty_count > 0;
    };
  spin_while_waiting(dummy_data_cb);
  ASSERT_EQ(string_count, 2);
  ASSERT_EQ(empty_count, 1);
}
