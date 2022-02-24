# Negotiated Publisher and Subscription implementation

This package implements the `NegotiatedPublisher` and `NegotiatedSubscription` classes.

## High-level description of negotiation process

From a high-level, what the negotiation process is trying to achieve is to create topics on the network that satisfy the preferences of the `NegotiatedPublisher` as well as all of the `NegotiatedSubscription`s.
From a practical standpoint, all of the decisions are made on the `NegotiatedPublisher`, which then informs the `NegotiatedSubscription`s of its decision.

The text below attempts to describe the general workings of the negotiation, and what users can expect.
For a full example, please see the [negotiated_examples](negotiated_examples) package.

## NegotiatedSubscription

### Construction

The `NegotiatedSubscription` constructor takes in 3 arguments: the node to use for subscriptions, the `topic_name`, and options that control some of the negotiation behaviors.
The constructor creates a subscription to the given `topic_name`, as well as a transient local publisher to `topic_name`/supported_types.
The subscription to the `topic_name` is used to inform the `NegotiatedPublisher` that this `NegotiatedSubscription` wants to be part of the negotiation, and is also the channel on which the `NegotiatedPublisher` will publish the results of the negotiation.
The transient local publisher is how this `NegotiatedSubscription` expresses its preferences to the `NegotiatedPublisher`.
It is transient local since we want the information to be available to late-joining `NegotiatedPublisher`s.
At this time, no information is actually provided on the `supported_types` publication since the user hasn't expressed any preferences to this `NegotiatedSubscription` yet.

### Adding supported types

After construction, the `NegotiatedSubscription` user should call `add_supported_callback` on the object to express preferences.
This API takes a template argument and 4 method arguments.
The template argument is the "supported type" that the user wants to express a preference for.
This supported type should have two fields in it: the ROS message type to support (as `MsgT`), along with a static string that contains arbitrary metadata about the supported type.
An example supported type looks like:

```
struct StringT
{
  using MsgT = std_msgs::msg::String;
  static const inline std::string supported_type_name = "a";
};
```

The method arguments are the weight for this particular type (expressed as a double), the quality-of-service that the subscription would prefer for this subscription, the callback to call if data for this type is delivered, and any `SubscriptionOptions` to apply if this subscription is created.

The user can call `add_supported_callbacks` as many times as necessary to express all preferences for this `NegotiatedSubscription`.
While the `NegotiatedSubscription` class is recording these preferences at this time, no data is sent to the publisher yet.

### Starting negotiation

Once all preferences have been expressed, the user should call `start` to begin negotiation.
At the time that `start` is called, all of the previously expressed preferences are collated together into a message, which is sent on the `topic_name`/supported_types transient local publication.
At this point, there are no further APIs for the user to call on the `NegotiatedSubscription` class.
All further work will be done behind the scenes automatically.

### Negotiation

Once `start` has been called, the `NegotiatedSubscription` is waiting for a `NegotiatedPublisher` to connect, collect subscriptions, negotiate between the preferences for those subscriptions, and send the results of that negotiation back.
When the `NegotiatedSubscription` class receives the results of that negotiation, it first looks to see if the negotiation was successful, and whether any of the chosen types is one of the ones it supports.
If either of those is false, nothing more happens and the `NegotiatedSubscription` continues waiting.
If the negotiation was successful and one of the chosen types is one it supports, then the `NegotiatedSubscription` creates a subscription to the chosen type.
If the `NegotiatedPublisher` sent back more than one chosen type that this `NegotiatedSubscription` supports, the first one in the list will be chosen.
Now when any publication of data to that topic happens, the callback that the user provided in `add_supported_callbacks` for that particular type will be called.
There is also the question of what happens during *re*-negotiation.
That is, it may be the case that the system has been transferring data for some time now, but a new `NegotiatedSubscription` comes online and expresses new preferences.
In that case, the `NegotiatedPublisher` will renegotiate, and send the results of that renegotiation to this `NegotiateSubscription`.
In response, this `NegotiatedSubscription` will again check if the negotiation was successful and whether any of the types it supports is in the list.
If both of those are true, the `NegotiatedSubscription` will look through the list to find a suitable match.
By default, the `NegotiatedSubscription` will attempt to continue to use the already connected subscription on a renegotiation.
If that is not available, then this `NegotiatedSubscription` will disconnect from the old topic, and connect to the new one.
If desired, the user can override this behavior by providing a different negotiated callback function when initially constructing the class.
That callback function should have a signature of:

```
negotiated_interfaces::msg::NegotiatedTopicInfo negotiate_cb(
  const negotiated_interfaces::msg::NegotiatedTopicInfo & existing_info,
  const negotiated_interfaces::msg::NegotiatedTopicsInfo & msg);

```

That is, it should take in a NegotiatedTopicInfo that contains details about the current connection, as well as a list of possible connections provided to the class by the `NegotiatedPublisher`.
If there is no current connection, all fields in `existing_info` will be default-initialized.
All of the `NegotiatedTopicInfo` structures in the msg list are guaranteed to be possible for this `NegotiatedSubscription`, that is, have been registered previously with `add_supported_callback`.
The callback should choose exactly one of the `NegotiatedTopicInfo` structures and return it.
If it could not choose for some reason, an empty `NegotiatedTopicInfo` should be returned.

## NegotiatedPublisher

### Construction

The `NegotiatedPublisher` constructor takes in 3 arguments: the node to use for publications, the `topic_name`, and options that control some of the negotiation behaviors.
The constructor creates a publisher to the given `topic_name`, as well as sets up some infrastructure to deal with graph change events (more on this later).
The publisher to the `topic_name` is used to gather the list of `NegotiatedSubscription`s that want to be part of this negotiation, and is also the channel on which this `NegotiatedPublisher` will publish the results of the negotiation.

### Adding supported types

After construction, the `NegotiatedPublisher` user should call `add_supported_type` on the object to express preferences.
This API takes a template argument and 3 method arguments.
The template argument is the "supported type" that the user wants to express a preference for.
This supported type should have two fields in it: the ROS message type to support (as `MsgT`), along with a static string that contains arbitrary metadata about the supported type.
An example supported type looks like:

```
struct StringT
{
  using MsgT = std_msgs::msg::String;
  static const inline std::string supported_type_name = "a";
};
```

The method arguments are the weight for this particular type (expressed as a double), the quality-of-service that the publisher would prefer for this publication, and any `PublisherOptions` to apply if this publisher is created.

The user can call `add_supported_type` as many times as necessary to express all preferences for this `NegotiatedPublisher`.
While the `NegotiatedPublisher` class is recording these preferences at this time, no negotiation is happening yet.

### Starting negotiation

Once all preferences have been expressed, the user should call `start` to begin negotiation.
At the time that `start` is called, the `NegotiatedPublisher` creates a transient local subscription to the topic_name/`supported_types` topic.
All of the data from any previously connected `NegotiatedSubscription`s will flow in at this time, and this `NegotiatedPublisher` will, by default, start running the negotiation algorithm.

### Negotiation

For each new publication to the topic_name/`supported_types` topic, the `NegotiatedPublisher` adds that `NegotiatedSubscription` to its internal map, and renegotiates with the entire network.
The negotiation attempts to find the lowest number of connections with the highest weight that will satisfy every member of the network.
By default it does this by exhaustively looking at all possible publication combinations.
That is, it first attempts to find a single publisher that will satisfy the whole network; if that fails, it tries to find 2 publishers that will satisfy the whole network, etc.
At each level (a level is the number of publishers), it will look through all combinations to find the highest weight one.
Once it finds at least one combination that works, it creates the appropriate number of publishers and informs all of the `NegotiatedSubscripton`s of the decision.

### Publishing data

If negotiation has produced a solution that works on the network, one or more of the preferences that the user expressed via `add_supported_type` are now active.
In order to find out which of the supported types is in use, the user can call `type_was_negotiated` with a template argument of the appropriate type.
If that particular type was one of the ones chosen in negotiation, true will be returned.

The user can then use the overloaded `publish` calls to publish data to the given type.
This is accomplished by creating a ROS 2 message as usual, then calling `publish` with the template argument corresponding to the type that was chosen.
For example:

```
if (type_was_chosen<StringT>()) {
  std_msg::msg::String msg;
  pub->publish<StringT>(msg);
}
```
