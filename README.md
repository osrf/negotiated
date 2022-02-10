# Negotiated Publishers and Subscriptions

This package implements the concepts of Negotiated Publishers and Subscriptions, as documented in https://github.com/ros-infrastructure/rep/blob/fcffcc41981b2dc658b81bff3a82373353d95ee4/rep-2009.rst .

In short, Negotiated Publishers and Subscriptions express "preferences" to each other on the ROS type and metadata associated with topics on the network.
The Negotiated Publisher than negotiates amongst all of the Negotiated Subscriptions in the network, and finds the "best" ROS type and metadata that satisfies all subscriptions.
This information is communicated from the Negotiated Publisher to all of the Negotiated Subscriptions, a new topic carrying data is created, and data starts to flow over that topic.

As a concrete example, imagine a Negotiated Publisher that can publish a sensor_msgs/msg/Image in either YUV420 or RGBA formats.
The publisher would prefer to send YUV420, since it can get that information most efficiently from hardware.
Also imagine a single connected Negotiated Subscription.
If that subscription supports YUV420, then the publisher will negotiate with the subscription for that, and data will be set most efficiently as YUV420.
On the other hand, if the subscription only supports RGBA, then the publisher will negotiate with the subscription for that, and data will still flow, just not as efficiently.

The simplified explanation above glosses over a lot of detail, but gives a general idea of what this set of packages is attempting to accomplish.
There is much more detail about this in REP-2009 (linked above), and in the [README](./negotiated/README.md) for the negotiation implementation.

There are 3 packages in this repository:

* negotiated_interfaces - These are the custom messages needed to communicate preferences and selections between NegotiatedPublishers and NegotiatedSubscriptions.
* negotiated - This is the implementation of the NegotiatedPublisher and NegotiatedSubscription classes.
* negotiated_examples - These are examples that use the NegotiatedPublisher and NegotiatedSubscription classes.
