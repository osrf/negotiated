# This is the message that is sent from a NegotiatedPublisher to all of the
# NegotiatedSubscriptions in a network to communicate which type was chosen
# after negotiation.  The NegotiatedSubscriptions will use this information to
# create their own subscriptions to the data.

# The ROS message type name that was chosen.
# This should be the canonical string name, like std_msgs/msg/String.
string ros_type_name

# The "supported" type name that was chosen.  This is the additional metadata
# that is combined with the ROS message type name above to create a unique
# supported type.  This is useful, for example, when trying to distinguish
# between a sensor_msgs/msg/Image that supports YUV and a sensor_msgs/msg/Image
# that supports RGB.
string supported_type_name

# The topic name that the NegotiatedPublisher created to publish data on.
# The NegotiatedSubscriptions will connect to this topic name.
string topic_name
