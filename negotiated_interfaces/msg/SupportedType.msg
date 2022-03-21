# This is the message that is sent from a NegotiatedSubscription to a
# NegotiatedPublisher to tell the publisher what types it supports.  The
# NegotiatedPublisher will use all of the supported types connected to the
# same topic to determine the best type to be used by all of the
# NegotiatedSubscriptions in the network.

# The ROS message type name that is supported.
# This should be the canonical string name, like std_msgs/msg/String.
string ros_type_name

# The "supported" type name.  This is essentially additional metadata that is
# combined with the ROS message type name above to create a unique supported
# type.  This is useful, for example, when trying to distinguish between a
# sensor_msgs/msg/Image that supports YUV and a sensor_msgs/msg/Image that
# supports RGB.
string supported_type_name

# The weight to assign to this supported type.  The higher the weight, the
# more likely that this type will be chosen by the publisher (though what
# ultimately gets chosen depends on the set of NegotiatedSubscriptions in
# the network.
float64 weight
