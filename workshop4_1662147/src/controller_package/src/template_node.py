#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
import time


class Node:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing ROS node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/endpoint",
            Float64,
            self.subscriber_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=1,
        )

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/endpoint",
            Float64,
            #Change buff size and queue size accordingly
            queue_size=1,
        )

        self.recvSequence = 0
        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_time)


    def subscriber_cb(self, data):
        if not self.initialized:
            return
        
        self.recvSequence += 1
        rospy.loginfo("Received message ({seqNumber}): {payload}".format(seqNumber=self.recvSequence,payload=data.data))
    
    def publish_time(self, event):
        msg = Float64()
        msg.data = time.time()

        self.publisher.publish(msg)
        self.sendSequence += 1
        
        rospy.loginfo("Message sent (({seqNumber})): {payload}".format(seqNumber=self.sendSequence,payload=msg))

if __name__ == "__main__":
    try:
        camera_node = Node(node_name = "template_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
