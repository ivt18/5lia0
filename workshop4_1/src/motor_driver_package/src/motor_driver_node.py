#!/usr/bin/env python3

import rospy
import time

from motor_driver_package.msg import MovementRequest

class MotorDriverNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing motor driver node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscriber
        self.subscriber = rospy.Subscriber(
            "/motors",
            MovementRequest,
            #Change buff size and queue size accordingly
            queue_size=10,
        )

        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Motor driver node initialized!")
    

if __name__ == "__main__":
    try:
        motor_driver_node = Node(node_name = "motor_driver_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
