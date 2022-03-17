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
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

#include "negotiated/negotiated_subscription.hpp"

namespace negotiated
{

namespace detail
{

using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

struct UpstreamNegotiatedSubscriptionHandle
{
  std::shared_ptr<negotiated::NegotiatedSubscription> subscription;
  std::shared_ptr<negotiated::NegotiatedSubscription::AfterSubscriptionCallbackHandle> handle;
};

/// Generate the key that is used as an index into the maps.
/**
 * Internal maps are keyed off of the uniqueness of individual types as given by
 * NegotiatedPublisher::add_supported_type().  This method is responsible for generating those
 * unique keys.
 *
 * \param[in] ros_type_name The canonical ROS type name, like std_msgs/msg/String.
 * \param[in] supported_type_name The arbitrary (but non-empty) string passed by a user.
 * \return A string that is the unique key into the maps.
 */
std::string generate_key(
  const std::string & ros_type_name,
  const std::string & supported_type_name);

struct SupportedTypeInfo final
{
  /// A map of PublisherGids to the weight associated with them.  Note that this is unique since
  /// every NegotiatedSubscription in the network has its own publisher for supported types.
  std::map<PublisherGid, double> gid_to_weight;

  /// The canonical ROS type name associated with this type.
  std::string ros_type_name;

  /// The arbitrary supported_type_name string associated with this type.
  std::string supported_type_name;

  /// Whether this supported type info is a compatible one (a regular publisher that the user
  /// gave to us), or a NegotiatedPublisher.
  bool is_compat;

  /// The factory function associated with this type.
  std::function<rclcpp::PublisherBase::SharedPtr(const std::string &)> pub_factory;

  /// The publisher created for this type; may be nullptr if this particular type was not chosen.
  std::shared_ptr<rclcpp::PublisherBase> publisher{nullptr};
};

/// The declaration of the default negotiation algorithm.
/**
 * This algorithm will exhaustively search for a solution that will satisfy all connected
 * NegotiatedSubscriptions.  If a solution exists, the solution with the lowest number of possible
 * publications is selected.  If no solution exists, or no solution exists with a lower number of
 * publishers than specified in 'maximum_solutions', an empty vector will be returned.
 *
 * \param[in] gid_set The set of Globally Unique Identifiers for NegotiatedSubscriptions that must
 *                    be satisfied for a full solution.
 * \param[in] key_to_supported_types A map of arbitrary, unique keys representing each possible
 *                                   negotiated type to a SupportedTypeInfo struct as above.
 * \param[in] maximum_solutions The maximum number of solutions to consider before giving up
 *                              negotiation.
 *
 * \return A vector of negotiated_interfaces::msg::SupportedType structures, one for each of the
 *         publishers that was chosen by negotiation.  If negotiation could not be completed for
 *         any reason, this vector may be empty.
 */
std::vector<negotiated_interfaces::msg::SupportedType> default_negotiation_callback(
  const std::map<detail::PublisherGid, std::vector<std::string>> & negotiated_sub_gid_to_keys,
  const std::map<std::string, SupportedTypeInfo> & key_to_supported_types,
  const std::unordered_set<std::shared_ptr<UpstreamNegotiatedSubscriptionHandle>>
  & upstream_negotiated_subscriptions,
  size_t maximum_solutions);

void default_update_downstream_cb(
  const std::map<std::string, detail::SupportedTypeInfo> & key_to_supported_types,
  const std::shared_ptr<std::map<detail::PublisherGid,
  std::vector<std::string>>> & negotiated_subscription_type_gids,
  const std::unordered_set<std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>> &
  upstream_negotiated_subscriptions,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_add,
  const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_remove,
  PublisherGid gid_key);

}  // namespace detail

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

  /// Whether to automatically disconnect publishers if the negotiation algorithm fails to find
  /// a solution to connect to all NegotiationSubscriptions in the network.  If set to false,
  /// then when a renegotiation fails existing publishers will be left as-is.
  bool disconnect_publishers_on_failure{true};

  /// The maximum number of solutions to allow while negotiating.  The default is to allow
  /// any number of solutions, but this may be restricted all the way down to 1 (passing
  /// 0 here will result in an exception while constructing).
  size_t maximum_negotiated_solutions{std::numeric_limits<size_t>::max()};

  /// The callback that will be called to perform negotiation.  The arguments are the same as
  /// those described for detail::default_negotiation_callback().
  std::function<std::vector<negotiated_interfaces::msg::SupportedType>(
      const std::map<detail::PublisherGid, std::vector<std::string>> &,
      const std::map<std::string, detail::SupportedTypeInfo> &,
      const std::unordered_set<std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>> &,
      size_t maximum_solutions)> negotiation_cb{detail::default_negotiation_callback};

  /// A callback that will be called to update the downstream NegotiatedSubscriptions (if there
  /// are any.
  std::function<void(
      const std::map<std::string, detail::SupportedTypeInfo> &,
      const std::shared_ptr<std::map<detail::PublisherGid,
      std::vector<std::string>>> &,
      const std::unordered_set<std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>> &,
      const negotiated_interfaces::msg::SupportedTypes &,
      const negotiated_interfaces::msg::SupportedTypes &,
      detail::PublisherGid)>
  update_downstream_cb{detail::default_update_downstream_cb};

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

  // Implement Rule of 5 here.  Particularly in cases where we have upstream
  // NegotiatedSubscriptions, a copy is hard to do reliably.  Mark copy constructors, move
  // constructors, and assignment operators as deleted.
  virtual ~NegotiatedPublisher();
  NegotiatedPublisher(const NegotiatedPublisher &) = delete;
  NegotiatedPublisher(NegotiatedPublisher &&) = delete;
  NegotiatedPublisher & operator=(const NegotiatedPublisher &) = delete;
  NegotiatedPublisher & operator=(NegotiatedPublisher &&) = delete;

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
    std::string key_name = detail::generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::invalid_argument("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, detail::SupportedTypeInfo());

    detail::PublisherGid gid{0};
    key_to_supported_types_[key_name].gid_to_weight[gid] = weight;
    key_to_supported_types_[key_name].ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type_name = T::supported_type_name;
    key_to_supported_types_[key_name].is_compat = false;
    key_to_supported_types_[key_name].pub_factory =
      [this, qos, options](const std::string & topic_name) -> rclcpp::PublisherBase::SharedPtr
      {
        return rclcpp::create_publisher<typename T::MsgT>(
          node_parameters_,
          node_topics_,
          topic_name,
          qos,
          options);
      };
  }

  /// Remove a supported type.
  /**
   * Remove a supported type from this NegotiatedPublisher.  The template argument must have the
   * same form as in add_supported_type(), and must have previously been added by
   * add_supported_type().
   *
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
    std::string key_name = detail::generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key_name) == 0) {
      throw std::invalid_argument("Specified key does not exist");
    }

    key_to_supported_types_.erase(key_name);
  }

  /// Add a compatible publisher to the negotiation.
  /**
   * These are publishers that are not created on-demand by negotiation, but instead have been
   * created by the API user (usually through rclcpp::create_publisher()).  These publishers
   * aren't actually involved in negotiation, but instead are appended to the results of negotiation.
   *
   * \param[in] pub - The rclcpp::Publisher shared_ptr to involve in negotiation.  A new reference
   *                  to this shared pointer will be taken, so in order to cleanup properly it must
   *                  be dropped with a call to remove_compatible_publisher().
   * \param[in] supported_type_name - An arbitrary name to give to the publisher.  This must be
   *                                  unique amongst all types given here, and must also match
   *                                  the name the subscriptions are expecting if it is to
   *                                  participate in negotiation.
   * \param[in] weight - The relative weight to assign this supported type; types with higher
   *                     weights are more likely to be chosen.
   * \throws std::invalid_argument if the given publisher is nullptr OR
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the exact same type has already been added.
   */
  template<typename MsgT>
  void add_compatible_publisher(
    std::shared_ptr<rclcpp::Publisher<MsgT>> pub,
    const std::string & supported_type_name,
    double weight)
  {
    if (pub == nullptr) {
      throw std::invalid_argument("The passed in publisher pointer must not be null");
    }

    if (supported_type_name.empty()) {
      throw std::invalid_argument("The supported_type_name cannot be empty");
    }

    using ROSMessageType = typename rclcpp::TypeAdapter<MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key_name = detail::generate_key(ros_type_name, supported_type_name);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::invalid_argument("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, detail::SupportedTypeInfo());

    detail::PublisherGid gid{0};
    key_to_supported_types_[key_name].gid_to_weight[gid] = weight;
    key_to_supported_types_[key_name].ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type_name = supported_type_name;
    key_to_supported_types_[key_name].is_compat = true;
    key_to_supported_types_[key_name].publisher = pub;
  }

  /// Remove a compatible publisher.
  /**
   * Remove a compatible publisher from this NegotiatedPublisher.  The publisher must have
   * previously been added by add_compatible_publisher().
   *
   * \param[in] pub - The rclcpp::Publisher shared_ptr to remove from negotiation.
   * \param[in] supported_type_name - The arbitrary name originally given to the compatible
   *                                  publisher.
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the type to be removed was not previously added.
   */
  template<typename MsgT>
  void remove_compatible_publisher(
    std::shared_ptr<rclcpp::Publisher<MsgT>> pub,
    const std::string & supported_type_name)
  {
    (void)pub;

    if (supported_type_name.empty()) {
      throw std::invalid_argument("The supported_type_name cannot be empty");
    }

    using ROSMessageType = typename rclcpp::TypeAdapter<MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key_name = detail::generate_key(ros_type_name, supported_type_name);
    if (key_to_supported_types_.count(key_name) == 0) {
      throw std::invalid_argument("Specified key does not exist");
    }

    key_to_supported_types_.erase(key_name);
  }

  /// Start tracking a NegotiatedSubscription as an upstream of this NegotiatedPublisher.
  /**
   * A NegotiatedSubscription added here will be considered "upstream" of this NegotiatedPublisher,
   * meaning that it has to successfully negotiate with its NegotiatedPublisher before this one
   * will even consider negotiating with its "downstream" NegotiatedSubscription(s).  This primarily
   * allows a NegotiatedPublisher/NegotiatedSubscription pair inside of the same rclcpp::Node, where
   * the results of the upstream negotiations have an effect on the downstream negotiations.
   *
   * \param[in] subscription A shared_ptr to the upstream NegotiatedSubscription to start tracking.
   */
  std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>
  add_upstream_negotiated_subscription(
    std::shared_ptr<negotiated::NegotiatedSubscription> subscription);

  /// Stop tracking a NegotiatedSubscription as an upstream of this NegotiatedPublisher.
  /**
   * Stop tracking a NegotiatedSubscription that was originally added via
   * add_upstream_negotiated_subscription().
   *
   * \param[in] subscription A shared_ptr to the upstream NegotiatedSubscription to stop tracking.
   */
  void remove_upstream_negotiated_subscription(
    const detail::UpstreamNegotiatedSubscriptionHandle * const handle);

  /// Start collecting information from the attached NegotiatedSubscriptions.
  /**
   * Until this method is called, no data from NegotiatedSubscriptions will be collected.
   * This is to give the user the chance to call add_supported_type() for each of the publisher
   * types to support.  Once that has been done, the user should call start().
   */
  void start();

  /// Negotiate between this NegotiatedPublisher and all connected NegotiatedSubscriptions.
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
    std::string key = detail::generate_key(ros_type_name, T::supported_type_name);
    if (key_to_supported_types_.count(key) == 0) {
      return 0;
    }

    return key_to_supported_types_[key].publisher != nullptr;
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
    std::string key = detail::generate_key(ros_type_name, T::supported_type_name);

    if (key_to_supported_types_.count(key) == 0) {
      RCLCPP_INFO(node_logging_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    std::shared_ptr<rclcpp::PublisherBase> publisher = key_to_supported_types_[key].publisher;
    if (publisher == nullptr) {
      RCLCPP_INFO(
        node_logging_->get_logger(),
        "This type hasn't been negotiated for, skipping publish");
      return;
    }

    auto pub = static_cast<rclcpp::Publisher<typename T::MsgT> *>(publisher.get());
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
    std::string key = detail::generate_key(ros_type_name, T::supported_type_name);

    if (key_to_supported_types_.count(key) == 0) {
      RCLCPP_INFO(node_logging_->get_logger(), "Negotiation hasn't happened yet, skipping publish");
      return;
    }

    std::shared_ptr<rclcpp::PublisherBase> publisher = key_to_supported_types_[key].publisher;
    if (publisher == nullptr) {
      RCLCPP_INFO(
        node_logging_->get_logger(),
        "This type hasn't been negotiated for, skipping publish");
      return;
    }

    auto pub = static_cast<rclcpp::Publisher<typename T::MsgT> *>(publisher.get());
    pub->publish(std::move(msg));
  }

  /// Stop the flow of data and forget about current negotiation.
  /**
   * The opposite of start(), this method can be called to stop the flow of all data
   * and have this NegotiatedPublisher forget about any negotiation.
   */
  void stop();

  /// Get the types supported by this NegotiatedPublisher.
  /**
   * Note that this is only the types as set by add_supported_type() and
   * add_compatible_publisher().  In particular, it does not contain any types from
   * upstream subscriptions.
   *
   * \return A map between the keys and SupportedTypeInfo structures that
   *         represent each of the supported types.
   */
  const std::map<std::string, detail::SupportedTypeInfo> & get_supported_types() const;

  /// Get the list of topics supported by this NegotiatedPublisher and all NegotiatedSubscriptions.
  /**
   * This list may be empty if negotiation has not yet happened, or failed.
   *
   * \return The list of negotiated topics supported by both this NegotiatedPublisher and
   *         all connected NegotiatedSubscriptions.
   */
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & get_negotiated_topics_info() const;

private:
  /// The timer callback to use to keep an eye on the network graph.
  /**
   * This method is run periodically to see if there have been changes in the network,
   * specifically NegotiatedSubscriptions that have dropped off.  If there are, they will be
   * removed from the set of participants in the negotiation, and, by default, renegotiation
   * will happen.
   */
  void graph_change_timer_callback();

  /// The callback that will be called once any upstream NegotiatedSubscriptions have successfully
  /// negotiated.
  void negotiate_on_upstream_success();

  void supported_types_cb(
    const negotiated_interfaces::msg::SupportedTypes & supported_types,
    const rclcpp::MessageInfo & msg_info);

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

  /// A map between unique type keys (as returned by detail::generate_key()) and SupportedTypeInfos.
  std::map<std::string, detail::SupportedTypeInfo> key_to_supported_types_;

  /// The publisher used to collect NegotiatedSubscriptions that are part of the network and for
  /// informing those NegotiatedSubscriptions of the chosen types.
  rclcpp::Publisher<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr
    negotiated_publisher_;

  /// The transient local subscription that gathers supported types from NegotiatedSubscriptions.
  rclcpp::Subscription<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_sub_;

  /// The timer used for checking for graph changes.
  rclcpp::TimerBase::SharedPtr graph_change_timer_;

  /// The graph event used for checking for graph changes.
  rclcpp::Event::SharedPtr graph_event_;

  /// The mutex to protect against concurrent modification of the
  /// negotiated_subscription_type_gids map.
  std::mutex negotiated_subscription_type_mutex_;

  /// A map between PublisherGids and the list of unique type keys (as returned by
  /// detail::generate_key()).  This is used to track which GIDs preferences have been met during
  /// negotiation.
  std::shared_ptr<std::map<detail::PublisherGid,
    std::vector<std::string>>> negotiated_subscription_type_gids_;

  /// A map to track any "upstream" negotiated subscriptions that need to be successful before
  /// this NegotiatedPublisher can negotiate with downstreams.
  std::unordered_set<std::shared_ptr<detail::UpstreamNegotiatedSubscriptionHandle>>
  upstream_negotiated_subscriptions_;

  /// A stored version of the negotiated topics that were sent to attached NegotiatedSubscriptions.
  /// This may be empty if negotiation has not happened yet or has failed.
  negotiated_interfaces::msg::NegotiatedTopicsInfo negotiated_topics_info_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_PUBLISHER_HPP_
