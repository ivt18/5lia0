#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64, UInt64

from motor_control_package.msg import Position, EncoderData

class PubNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.publisher_fusion = rospy.Publisher(
            "/camera/fusion",
            Float64,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.publisher_position = rospy.Publisher(
            "/motor_control/position",
            Position,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.publisher_tof = rospy.Publisher(
            "/motor_control/tof",
            UInt64,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer1 = rospy.Timer(rospy.Duration(0.05), self.publish_position)
        self.timer2 = rospy.Timer(rospy.Duration(0.05), self.publish_tof)
        self.timer3 = rospy.Timer(rospy.Duration(0.05), self.publish_fusion)


    def publish_position(self, event):
        msg = Postion()
        msg.x = 0
        msg.y = 0
        msg.theta = 0

        self.publisher_position.publish(msg)

    def publish_tof(self, event):
        msg = UInt64()
        msg.data = 0.15
        
        self.publisher_tof.publish(msg)

    def publish_fusion(self, event)
        msg = Float64()
        msg.data = 0

        self.publisher_fusion.publish(msg)


if __name__ == "__main__":
    try:
        node = PubNode(node_name = "test_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
