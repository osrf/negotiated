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

#ifndef NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
#define NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

/// NegotiatedPublisherOptions allows the user to control some aspects of the NegotiatedPublisher.
struct NegotiatedPublisherOptions final
{
  /// Whether to automatically cause a renegotiation when NegotiatedSubscriptions leave the graph.
  /// If set to false, then renegotiations will only happen when the user calls
  /// NegotiatedPublisher::negotiate().
  bool negotiate_on_subscription_removal{true};

  /// Whether to automatically cause a renegotiation when NegotiatedSubscriptions join the graph.
  /// If set to false, then renegotiations will only happen when the user calls
  /// NegotiatedPublisher::negotiate().
  bool negotiate_on_subscription_add{true};

  /// Whether to allow a multiple-topic solution during negotiation.  If set to false, then
  /// only single-topic solutions will be considered during negotation, and if one can't be
  /// found negotiation will fail.
  bool allow_multiple_types{true};

  /// A callback that will be called if negotiation is successful.  This gives the
  /// NegotiatedPublisher user a chance to react in arbitrary ways once negotiation has happened.
  std::function<void(const negotiated_interfaces::msg::NegotiatedTopicsInfo &)>
  successful_negotiation_cb{nullptr};
};

/// NegotiatedPublisher implements the publishing side of a negotiated system.
/// It is ultimately responsible for determining which publishers and subscriptions in a system
/// get created.
class NegotiatedPublisher
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedPublisher)

  /// Create a new NegotiatedPublisher with the given "base" topic_name.
  /**
   * The topic_name given here will be used to determine the list of NegotiatedSubscriptions in the
   * network.  It will also be used to communicate the chosen topic_name at the end of negotiation.
   *
   * \param[in] node_parameters The node parameters interface to use.
   * \param[in] node_topics The node topics interface to use.
   * \param[in] node_logging The node logging interface to use.
   * \param[in] node_graph The node graph interface to use.
   * \param[in] node_base The node base interface to use.
   * \param[in] node_timers The node timers interface to use.
   * \param[in] topic_name The topic name to use for the "base" topic and as the prefix for the
   *                       final topic name.
   * \param[in] negotiated_pub_options The options to use.
   */
  explicit NegotiatedPublisher(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers,
    const std::string & topic_name,
    const NegotiatedPublisherOptions & negotiated_pub_options = NegotiatedPublisherOptions());

  /// Create a new NegotiatedPublisher with the given "base" topic_name.
  /**
   * The topic_name given here will be used to determine the list of NegotiatedSubscriptions in the
   * network.  It will also be used to communicate the chosen topic_name at the end of negotiation.
   *
   * \param[in] node The node to use to create the publishers and subscriptions.
   * \param[in] topic_name The topic name to use for the "base" topic and as the prefix for the
   *                       final topic name.
   * \param[in] negotiated_pub_options The options to use.
   */
  template<typename NodeT>
  explicit NegotiatedPublisher(
    NodeT & node,
    const std::string & topic_name,
    const NegotiatedPublisherOptions & negotiated_pub_options = NegotiatedPublisherOptions())
  : NegotiatedPublisher(
      node.get_node_parameters_interface(),
      node.get_node_topics_interface(),
      node.get_node_logging_interface(),
      node.get_node_graph_interface(),
      node.get_node_base_interface(),
      node.get_node_timers_interface(),
      topic_name,
      negotiated_pub_options)
  {
  }

  /// Add a supported type with a weight.
  /**
   * Add a supported type to this NegotiatedPublisher.  The template argument must have the form:
   *
   * struct MyType {
   *   using MsgT = my_ros::msg::type;
   *   static const inline std::string supported_type_name = "arbitrary";
   * };
   *
   * As a concrete example:
   *
   * struct MyType {
   *   using MsgT = std_msgs::msg::String;
   *   static const inline std::string supported_type_name = "arbitrary";
   * };
   *
   * \param[in] weight - The relative weight to assign this supported type; types with higher
   *                     weights are more likely to be chosen.
   * \param[in] qos - The Quality of Service parameters to apply if this type is chosen.
   * \param[in] options - The PublisherOptions to apply if this type is chosen.
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the exact same type has already been added.
   */
  template<typename T>
  void add_supported_type(
    double weight,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions())
  {
    if (T::supported_type_name.empty()) {
      throw std::invalid_argument("The supported_type_name cannot be empty");
    }

    using ROSMessageType = typename rclcpp::TypeAdapter<typename T::MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key_name = generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::invalid_argument("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, SupportedTypeInfo());

    auto factory =
      [this, qos, options](const std::string & topic_name) -> rclcpp::PublisherBase::SharedPtr
      {
        return rclcpp::create_publisher<typename T::MsgT>(
          node_parameters_,
          node_topics_,
          topic_name,
          qos,
          options);
      };
    key_to_supported_types_[key_name].pub_factory = factory;

    PublisherGid gid{0};
    key_to_supported_types_[key_name].gid_to_weight[gid] = weight;
    key_to_supported_types_[key_name].ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type_name = T::supported_type_name;
  }

  /// Remove a supported type.
  /**
   * Remove a supported type from this NegotiatedPublisher.  The template argument must have the
   * same form as in add_supported_type(), and must have previously been added by
   * add_supported_type().
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the type to be removed was not previously added.
   */
  template<typename T>
  void remove_supported_type()
  {
    if (T::supported_type_name.empty()) {
      throw std::invalid_argument("The supported_type_name cannot be empty");
    }

    using ROSMessageType = typename rclcpp::TypeAdapter<typename T::MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key_name = generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) == 0) {
      throw std::invalid_argument("Specified key does not exist");
    }

    key_to_supported_types_.erase(key_name);
  }

  /// Start collecting information from the attached NegotiatedSubscriptions.
  /**
   * Until this method is called, no data from NegotiatedSubscriptions will be collected.
   * This is to give the user the chance to call add_supported_type() for each of the publisher
   * types to support.  Once that has been done, the user should call start().
   */
  void start();

  /// Negotiate between this NegotiatedPublisher and all connected NegotatiedSubscriptions.
  /**
   * This method can be called by the user at any time to negotiate or renegotiate.
   * If the default values for NegotiatedPublisherOptions are taken, this will automatically
   * get called any time a NegotiatedSubscriptions joins or leaves the network.
   */
  void negotiate();

  /// Determine whether a particular type was chosen by negotiation.
  /**
   * This method allows the user to determine whether a particular type registered
   * with add_supported_type() has been chosen by negotiation.  Note that negotiation may choose
   * more than one type.  The template argument here should be one of the same ones
   * as used in add_supported_type().
   *
   * \return Whether the particular type was chosen by negotiation.
   */
  template<typename T>
  bool type_was_negotiated()
  {
    using ROSMessageType = typename rclcpp::TypeAdapter<typename T::MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key = generate_key(ros_type_name, T::supported_type_name);
    return key_to_publisher_.count(key) > 0;
  }

  /// Publish constref data to a chosen publisher.
  /**
   * This method allows the user to publish data using constref to a chosen publisher.
   * This will only succeed if a publisher of that particular type has been chosen by
   * negotiation; call type_was_negotiated() first to determine this.
   * If a publisher of that particular type was not chosen, this method returns without doing
   * any work.
   * If a publisher of that particular type was chosen, this method publishes the data as
   * requested.
   *
   * \param[in] msg The message to publish.
   */
  template<typename T, typename MessageT>
  void publish(const MessageT & msg)
  {
    using ROSMessageType = typename rclcpp::TypeAdapter<typename T::MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key = generate_key(ros_type_name, T::supported_type_name);

    if (key_to_publisher_.count(key) == 0) {
      RCLCPP_INFO(node_logging_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    std::shared_ptr<rclcpp::PublisherBase> publisher_ = key_to_publisher_[key];

    auto pub = static_cast<rclcpp::Publisher<typename T::MsgT> *>(publisher_.get());
    pub->publish(msg);
  }

  /// Publish std::unique_ptr data to a chosen publisher.
  /**
   * This method allows the user to publish data using constref to a chosen publisher.
   * This will only succeed if a publisher of that particular type has been chosen by
   * negotiation; call type_was_negotiated() first to determine this.
   * If a publisher of that particular type was not chosen, this method returns without doing
   * any work.
   * If a publisher of that particular type was chosen, this method publishes the data as
   * requested.
   *
   * \param[in] msg The message to publish.
   */
  template<typename T, typename MessageT>
  void publish(std::unique_ptr<MessageT> msg)
  {
    using ROSMessageType = typename rclcpp::TypeAdapter<typename T::MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key = generate_key(ros_type_name, T::supported_type_name);

    if (key_to_publisher_.count(key) == 0) {
      RCLCPP_INFO(node_logging_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    std::shared_ptr<rclcpp::PublisherBase> publisher_ = key_to_publisher_[key];

    auto pub = static_cast<rclcpp::Publisher<typename T::MsgT> *>(publisher_.get());
    pub->publish(std::move(msg));
  }

  /// Stop the flow of data and forget about current negotiation.
  /**
   * The opposite of start(), this method can be called to stop the flow of all data
   * and have this NegotiatedPublisher forget about any negotiation.
   */
  void stop();

private:
  using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

  struct SupportedTypeInfo final
  {
    /// A map of PublisherGids to the weight associated with them.  Note that this is unique since
    /// every NegotiatedSubscription in the network has its own publisher for suppported types.
    std::map<PublisherGid, double> gid_to_weight;
    /// The canonical ROS type name associated with this type.
    std::string ros_type_name;
    /// The arbitrary supported_type_name string associated with this type.
    std::string supported_type_name;
    /// The factory function associated with this type.
    std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)> pub_factory;
  };

  /// The timer callback to use to keep an eye on the network graph.
  /**
   * This method is run periodically to see if there have been changes in the network,
   * specifically NegotiatedSubscriptions that have dropped off.  If there are, they will be
   * removed from the set of participants in the negotiation, and, by default, renegotiation
   * will happen.
   */
  void timer_callback();

  /// Generate the key that is used as an index into the maps.
  /**
   * Two of the internal maps are keyed off of the uniqueness of individual types as given by
   * add_supported_type().  This method is responsible for generating those unique keys.
   *
   * \param[in] ros_type_name The canonical ROS type name, like std_msgs/msg/String.
   * \param[in] supported_type_name The arbitrary (but non-empty) string passed by a user.
   * \return A string that is the unique key into the maps.
   */
  std::string generate_key(
    const std::string & ros_type_name,
    const std::string & supported_type_name);

  /// The node parameters interface to use.
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  /// The node topics interface to use.
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  /// The node logging interface to use.
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  /// The node graph interface to use.
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_;
  /// The node base interface to use.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  /// The node timers interface to use.
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  /// The original topic_name provided by the user.
  std::string topic_name_;
  /// The original options to this class provided by the user.
  NegotiatedPublisherOptions negotiated_pub_options_;

  /// A map between unique type keys (as returned by generate_key()) and SupportedTypeInfos.
  std::map<std::string, SupportedTypeInfo> key_to_supported_types_;
  /// The publisher used to collect NegotiatedSubscriptions that are part of the network and for
  /// informing those NegotiatedSubscriptions of the chosen types.
  rclcpp::Publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr
    negotiated_publisher_;
  /// A map between unique type keys (as returned by generate_key()) and an associated
  /// rclcpp::Publisher.  Will only be populated if the particular type was chosen by negotiation.
  std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> key_to_publisher_;
  /// The transient local subscription that gathers supported types from NegotiatedSubscriptions.
  rclcpp::Subscription<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_sub_;
  /// The timer used for checking for graph changes.
  rclcpp::TimerBase::SharedPtr graph_change_timer_;
  /// The graph event used for checking for graph changes.
  rclcpp::Event::SharedPtr graph_event_;
  /// The mutex to protect against concurrent modification of the
  /// negotiated_subscription_type_gids map.
  std::mutex negotiated_subscription_type_mutex_;
  /// A map between PublisherGids and the list of unique type keys (as returned by generate_key()).
  /// This is used to track which GIDs preferences have been met during negotiation.
  std::shared_ptr<std::map<PublisherGid,
    std::vector<std::string>>> negotiated_subscription_type_gids_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
