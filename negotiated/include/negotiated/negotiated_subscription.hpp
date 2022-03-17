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

#ifndef NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
#define NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_

#include <functional>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "negotiated_interfaces/msg/negotiated_topic_info.hpp"
#include "negotiated_interfaces/msg/negotiated_topics_info.hpp"
#include "negotiated_interfaces/msg/supported_type.hpp"
#include "negotiated_interfaces/msg/supported_types.hpp"

namespace negotiated
{

namespace detail
{

/// The callback that will be called when a NegotiatedPublisher sends the results of negotiation.
/**
 * This is the function that will determine which choice of the NegotiatedPublisher this particular
 * NegotiatedSubscription is going to choose.  If this NegotiatedSubscription is already connected
 * to a particular topic, this callback will prefer to choose that topic again if available.
 * Otherwise, this callback will choose the first topic in the list that is supported by
 * this NegotiatedSubscription.
 *
 * \param[in] existing_info The information about the existing subscription; may be empty
 * \param[in] msg The list of topics that the NegotiatedPublisher chose.
 * \return The topic that this NegotiatedSubscription should connect to.  May be empty, in which
 *         case no connection will be made.
 */
negotiated_interfaces::msg::NegotiatedTopicInfo default_negotiate_cb(
  const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg);

}  // namespace detail

/// NegotiatedSubscriptionOptions allows the user to control some aspects of NegotiatedSubscription.
struct NegotiatedSubscriptionOptions
{
  /// Whether to automatically disconnect the NegotiatedSubscription on negotiation failure.
  /// If set to false, will keep subscription active to a topic even if the NegotiatedPublisher
  /// has failed renegotiation.
  bool disconnect_on_negotiation_failure{true};

  /// A user settable callback to call instead of the default_negotiate_cb().
  std::function<negotiated_interfaces::msg::NegotiatedTopicInfo(
      const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
      const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg)> negotiate_cb{
    detail::default_negotiate_cb};
};

/// NegotiatedSubscription implements the subscription side of a negotiated system.
/// It expresses preferences to the NegotiatedPublisher, and then chooses which option to
/// take after the NegotiatedPublisher negotiates.
class NegotiatedSubscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(NegotiatedSubscription)

  using AfterSubscriptionCallbackFunction = std::function<void ()>;
  struct AfterSubscriptionCallbackHandle
  {
    AfterSubscriptionCallbackFunction callback;
  };

  using PublisherGid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

  struct SupportedTypeInfo final
  {
    /// The supported type info associated with this type.
    negotiated_interfaces::msg::SupportedType supported_type;

    /// Whether this supported type is a compatible subscription, i.e. one created outside of this
    /// NegotiatedSubscription.
    bool is_compat;

    /// The saved subscription pointer, which is used in the case that this is a compatible
    /// subscription.
    rclcpp::SubscriptionBase::SharedPtr subscription;

    PublisherGid gid;

    /// The factory function associated with this type.
    std::function<rclcpp::SubscriptionBase::SharedPtr(const std::string &)> sub_factory;
  };

  /// Create a new NegotiatedSubscription with the given "base" topic_name.
  /**
   * The topic_name given here will be used to initially contact the NegotiatedPublisher.  It will
   * also be used to get information about the chosen topic_names after negotiation.
   *
   * \param[in] node_parameters The node parameters interface to use.
   * \param[in] node_topics The node topics interface to use.
   * \param[in] node_logging The node logging interface to use.
   * \param[in] topic_name The topic name to use for the "base" topic and as the prefix for the
   *                       final topic name.
   * \param[in] negotiated_sub_options The options to use.
   */
  explicit NegotiatedSubscription(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & topic_name,
    const NegotiatedSubscriptionOptions & negotiated_sub_options = NegotiatedSubscriptionOptions());

  /// Create a new NegotiatedSubscription with the given "base" topic_name.
  /**
   * The topic_name given here will be used to initially contact the NegotiatedPublisher.  It will
   * also be used to get information about the chosen topic_names after negotiation.
   *
   * \param[in] node The node to use to create the publishers and subscriptions.
   * \param[in] topic_name The topic name to use for the "base" topic and as the prefix for the
   *                       final topic name.
   * \param[in] negotiated_sub_options The options to use.
   */
  template<typename NodeT>
  explicit NegotiatedSubscription(
    NodeT & node,
    const std::string & topic_name,
    const NegotiatedSubscriptionOptions & negotiated_sub_options = NegotiatedSubscriptionOptions())
  : NegotiatedSubscription(
      node.get_node_parameters_interface(),
      node.get_node_topics_interface(),
      node.get_node_logging_interface(),
      topic_name,
      negotiated_sub_options)
  {
  }

  /// Add a supported callback with a weight.
  /**
   * Add a supported callback to this NegotiatedSubscription.  The template argument must have the form:
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
   * \param[in] callback - The user callback to call if data starts flowing on this topic.
   * \param[in] options - The SubscriptionOptions to apply if this type is chosen.
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the exact same type has already been added.
   */
  template<typename T, typename CallbackT>
  void add_supported_callback(
    double weight,
    const rclcpp::QoS & qos,
    CallbackT callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
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
    key_to_supported_types_[key_name].supported_type.ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type.supported_type_name = T::supported_type_name;
    key_to_supported_types_[key_name].supported_type.weight = weight;
    key_to_supported_types_[key_name].is_compat = false;
    key_to_supported_types_[key_name].sub_factory =
      [this, qos, callback,
        options](const std::string & topic_name) -> rclcpp::SubscriptionBase::SharedPtr
      {
        return rclcpp::create_subscription<typename T::MsgT>(
          node_parameters_,
          node_topics_,
          topic_name,
          qos,
          callback,
          options);
      };
  }

  /// Remove a supported callback.
  /**
   * Remove a supported callback from this NegotiatedSubscription.  The template argument must have the
   * same form as in add_supported_callback(), and must have previously been added by
   * add_supported_callback().
   *
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the type to be removed was not previously added.
   */
  template<typename T>
  void remove_supported_callback()
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

    if (ros_type_name == existing_topic_info_.ros_type_name &&
      T::supported_type_name == existing_topic_info_.supported_type_name)
    {
      // We just removed the one we are connected to, disconnect
      existing_topic_info_.ros_type_name = "";
      existing_topic_info_.supported_type_name = "";
      existing_topic_info_.topic_name = "";
      subscription_.reset();
    }
  }

  /// Add a compatible subscription with a weight.
  /**
   * Add a compatible subscription to this NegotiatedSubscription.  A compatible subscription is
   * one that has already been created by the user (using the normal rclcpp methods), but is
   * intended as a fallback option in case negotiation fails or no NegotiatedPublisher ever
   * attaches.  Note that there can only ever be one of these in this NegotiatedSubscription.
   *
   * \param[in] sub - The already created subscription to add as a compatible subscription.
   * \param[in] supported_type_name - The supported type name to associate with this compatible
   *                                  subscription.
   * \param[in] weight - The relative weight to assign this supported type; types with higher
   *                     weights are more likely to be chosen.
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the exact same type has already been added.
   */
  template<typename MsgT>
  void add_compatible_subscription(
    std::shared_ptr<rclcpp::Subscription<MsgT>> sub,
    const std::string & supported_type_name,
    double weight)
  {
    if (sub == nullptr) {
      throw std::invalid_argument("The passed in subscription pointer must not be null");
    }

    if (supported_type_name.empty()) {
      throw std::invalid_argument("The supported_type_name cannot be empty");
    }

    if (current_subscription_is_compat_) {
      throw std::invalid_argument(
              "The current subscription is already compat; remove that one first");
    }

    using ROSMessageType = typename rclcpp::TypeAdapter<MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key_name = generate_key(ros_type_name, supported_type_name);
    if (key_to_supported_types_.count(key_name) != 0) {
      throw std::invalid_argument("Cannot add duplicate key to supported types");
    }

    key_to_supported_types_.emplace(key_name, SupportedTypeInfo());
    key_to_supported_types_[key_name].supported_type.ros_type_name = ros_type_name;
    key_to_supported_types_[key_name].supported_type.supported_type_name = supported_type_name;
    key_to_supported_types_[key_name].supported_type.weight = weight;
    key_to_supported_types_[key_name].is_compat = true;
    key_to_supported_types_[key_name].subscription = sub;
    key_to_supported_types_[key_name].sub_factory = nullptr;

    // If we have already previously negotiated for something else, then we will *not* replace
    // it here and instead will just put this in the list as something that could be negotiated
    // for.  If nothing has been negotiated yet, then we will automatically use it.
    if (subscription_ == nullptr) {
      existing_topic_info_.ros_type_name = ros_type_name;
      existing_topic_info_.supported_type_name = supported_type_name;
      existing_topic_info_.topic_name = sub->get_topic_name();
      subscription_ = sub;
      current_subscription_is_compat_ = true;
    }
  }

  /// Remove a compatible subscription.
  /**
   * Remove a compatible publisher from this NegotiatedSubscription.  The subscription must have
   * previously been added by add_compatible_subscription().
   *
   * \param[in] sub - The rclcpp::Subscription shared_ptr to remove from negotiation.
   * \param[in] supported_type_name - The arbitrary name originally given to the compatible
   *                                  subscription.
   * \throws std::invalid_argument if the supported_type_name in the structure is the empty string, OR
   * \throws std::invalid_argument if the type to be removed was not previously added.
   */
  template<typename MsgT>
  void remove_compatible_subscription(
    std::shared_ptr<rclcpp::Subscription<MsgT>> sub,
    const std::string & supported_type_name)
  {
    (void)sub;

    if (supported_type_name.empty()) {
      throw std::invalid_argument("The supported_type_name cannot be empty");
    }

    using ROSMessageType = typename rclcpp::TypeAdapter<MsgT>::ros_message_type;
    std::string ros_type_name = rosidl_generator_traits::name<ROSMessageType>();
    std::string key_name = generate_key(ros_type_name, supported_type_name);
    if (key_to_supported_types_.count(key_name) == 0) {
      throw std::invalid_argument("Specified key does not exist");
    }

    key_to_supported_types_.erase(key_name);

    if (current_subscription_is_compat_) {
      existing_topic_info_.ros_type_name = "";
      existing_topic_info_.supported_type_name = "";
      existing_topic_info_.topic_name = "";
      subscription_ = nullptr;
      current_subscription_is_compat_ = false;
    }
  }

  /// Add and remove supported types from a downstream NegotiatedPublisher.
  /**
   * \param[in] downstream_types_to_add The types to track as part of a downstream
   *                                    NegotiatedPublisher.
   * \param[in] downstream_types_to_remove The types to stop tracking as part of a downstream
   *                                       NegotiatedPublisher.
   * \param[in] gid The GID of the NegotiatedSubscription to track.
   */
  void update_downstream_supported_types(
    const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_add,
    const negotiated_interfaces::msg::SupportedTypes & downstream_types_to_remove,
    const PublisherGid & gid);

  /// Remove all supported types from a downstream NegotiatedPublisher.
  /**
   * \param[in] downstream_types The SupportedTypes list of downstream types to remove.
   */
  void remove_all_downstream_supported_types(
    const negotiated_interfaces::msg::SupportedTypes & downstream_types);

  /// Start sending preferences to the NegotiatedPublisher.
  /**
   * This is separated from the constructor to give the user time to call add_supported_callback()
   * to express all preferences.  Once that is done, the user should call start.  This may be called
   * more than once to express new preferences to the NegotiatedPublisher, which will generally
   * cause a renegotiation.
   */
  void start();

  /// Get the publisher count on the negotiated topic.
  /**
   * This is the topic used for negotiation, not for data.
   *
   * \return The number of publishers on the negotiated topic.
   */
  size_t get_negotiated_topic_publisher_count() const;

  /// Get the publisher count on the data topic.
  /**
   * This is the topic used for transferring data.  This may be zero in the case that no data topic
   * has yet been negotiated.
   *
   * \return The number of publishers on the data topic.
   */
  size_t get_data_topic_publisher_count() const;

  /// Get the list of topics supported by this NegotiatedSubscription and the NegotiatedPublisher.
  /**
   * This list may be empty if negotiation has not yet happened, or failed.
   *
   * \return The list of negotiated topics supported by both this NegotiatedSubscription and
   *         the connected NegotiatedPublisher.
   */
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & get_negotiated_topics_info() const;

  /// Add a callback to be called after a subscription is negotiated and created.
  /**
   * This is primarily intended for internal use by the NegotiatedPublisher.  Using it for other
   * purposes may break the use-case of a NegotiatedPublisher and NegotiatedSubscription in the
   * same rclcpp::Node; use it at your own risk!
   *
   * \param[in] cb The callback to call after a subscription is successfully negotiated and created.
   */
  std::shared_ptr<AfterSubscriptionCallbackHandle> add_after_subscription_callback(
    const AfterSubscriptionCallbackFunction & cb);

  /// Remove the after subscription callback.
  /**
   * Like add_after_subscription_callback(), this is primarily intended for internal use by the
   * NegotiatedPublisher.  Using it for other purposes may break the use-case of a
   * NegotiatedPublisher and NegotiatedSubscription in the same rclcpp::Node; use it at your
   * own risk!
   *
   * \param[in] cb The callback to remove.
   */
  void remove_after_subscription_callback(const AfterSubscriptionCallbackHandle * const handle);

  /// Get the types supported by this NegotiatedSubscription.
  /**
   * Note that this is only the types as set by add_supported_callback() and
   * add_compatible_subscription().  In particular, it does not contain any types from
   * downstream publishers; see get_downstream_key_to_supported_types() for downstream types.
   *
   * \return An unordered map between the keys and SupportedTypeInfo structures that
   *         represent each of the supported types.
   */
  const std::unordered_map<std::string, SupportedTypeInfo> & get_supported_types() const;

  /// Get the info about the negotiated topic that was chosen.
  /**
   * This is the stored version of the topic that was chosen by negotiation between this
   * NegotiatedSubscription and it's upstream NegotiatedPublisher.  Note that in the case that
   * negotiation hasn't happened yet, or failed for some reason, this will be a default-initialized
   * structure (i.e. the structure fields of ros_type_name, supported_type_name, and topic_name
   * will all be the empty string).
   *
   * \return A NegotiatedTopicInfo structure containing the topic that was chosen by negotiation.
   */
  const negotiated_interfaces::msg::NegotiatedTopicInfo & get_existing_topic_info() const;

  /// Get the downstream types supported by this NegotiatedSubscription.
  /**
   * These are only the types that downstream NegotiatedPublishers support.  If there are no
   * downstream NegotiatedPublishers, or they haven't negotiated yet, this may be an empty map.
   * To get types supported by this NegotiatedSubscription, call get_supported_types().
   *
   * \return An unordered map between the keys and SupportedTypeInfo structures that represent
   *         each of the downstream types.
   */
  const std::unordered_map<std::string, std::vector<SupportedTypeInfo>> &
  get_downstream_key_to_supported_types() const;

private:
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

  /// The callback that is called when the NegotiatedPublisher sends the results of a negotiation.
  /**
   * This method will filter the results to only the ones that this object can possibly support
   * (based on the types registered with add_supported_callback()), and then call the negotiate_cb
   * in the options structure.  If the returned type from the negotiated_cb is empty, this will
   * disconnect from the existing subscription (by default).  If the returned type is not empty,
   * but is the same as the existing connection, no further action will be taken.
   * If the returned type is different than the existing connection, then the existing connection
   * will be broken and a new connection will be made.
   *
   * \param[in] msg The NegotiatedTopicsInfo that contains the results of negotiation.
   */
  void topics_info_cb(const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg);

  /// Send our preferences to any connected NegotiatedPublisher.
  /**
   * This is separated from the call to start() so that we can separate whether the user told
   * us to start negotiating from any downstream information we may be given.
   */
  void send_preferences();

  /// The node parameters interface to use.
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  /// The node topics interface to use.
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;

  /// The node logging interface to use.
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;

  /// The original options to this class provided by the user.
  NegotiatedSubscriptionOptions negotiated_sub_options_;

  /// A map between unique type keys (as returned by generate_key()) and the SupportedTypeInfos.
  std::unordered_map<std::string, SupportedTypeInfo> key_to_supported_types_;

  /// Whether the user has called start() yet.
  bool user_called_start_{false};

  /// A map of any downstream types between unique type keys (as returned by generate_key()) and
  /// the SupportedTypeInfos.
  std::unordered_map<std::string,
    std::vector<SupportedTypeInfo>> downstream_key_to_supported_types_;

  /// The subscription used to include this class in the NegotiatedPublisher network and to
  /// receive preferences from the NegotiatedPublisher once they have been negotiated.
  rclcpp::Subscription<negotiated_interfaces::msg::NegotiatedTopicsInfo>::SharedPtr
    negotiated_subscription_;

  /// Whether the current subscription we are using is a compatible one or not.
  bool current_subscription_is_compat_{false};

  /// The subscription that data flows over once negotiation has happened.
  std::shared_ptr<rclcpp::SubscriptionBase> subscription_{nullptr};

  /// The transient local publisher that informs the NegotiatedPublisher of preferences.
  rclcpp::Publisher<negotiated_interfaces::msg::SupportedTypes>::SharedPtr supported_types_pub_;

  /// Saved information about the currently connected data topic.
  negotiated_interfaces::msg::NegotiatedTopicInfo existing_topic_info_;

  /// A list of all of the topics that the publisher sent to us, that we also support.
  negotiated_interfaces::msg::NegotiatedTopicsInfo negotiated_topics_;

  /// A list of callbacks to be called after a subscription has successfully been created.
  std::list<std::shared_ptr<AfterSubscriptionCallbackHandle>> after_subscription_cbs_;
};

}  // namespace negotiated

#endif  // NEGOTIATED__NEGOTIATED_SUBSCRIPTION_HPP_
