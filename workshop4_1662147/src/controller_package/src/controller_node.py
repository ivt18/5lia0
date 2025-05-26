#!/usr/bin/env python3

import time

import numpy as np
import rospy
from controller_package.msg import MovementRequest


class ControllerNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing controller node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/commands",
            MovementRequest,
            queue_size=10,
        )

        self.initialized = True

        rospy.loginfo("Controller node initialized!")

        while True:
            request_type = int(input("request type: "))
            value = float(input("value: "))

            msg = MovementRequest()

            msg.request_type = request_type
            msg.value = value
            self.publisher.publish(msg)
        

if __name__ == "__main__":
    try:
        controller_node = ControllerNode("controller_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
