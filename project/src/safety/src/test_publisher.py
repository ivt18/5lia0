#!/usr/bin/env python2

import rospy
from std_msgs.msg import Bool

from enums import StopGo


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {self.node_name} node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/camera/text",
            Bool,
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("Node {name} initialized!".format(name=self.node_name))
            
        while True:
            # value = bool(input("signal: "))
            value = False

            msg = Bool()
            msg.data = value
            
            self.publisher.publish(msg)
            # rospy.loginfo("Message sent: {og} -> {signal}".format(og=msg.data, signal="GO" if msg.data == StopGo.GO else "STOP"))
            rospy.loginfo("Message sent: {signal}".format(signal=StopGo(msg.data)))


if __name__ == "__main__":
    try:
        node = Node(node_name = "test_publisher")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
