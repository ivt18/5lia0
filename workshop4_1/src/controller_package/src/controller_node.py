#!/usr/bin/env python3

import rospy
import time

from msg import MovementRequest


class ControllerNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing controller node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/motors",
            MovementRequest,
            #Change buff size and queue size accordingly
            queue_size=10,
        )

        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Controller node initialized!")

        while True:
            action = input("desired action: ")
            distance = input("distance: ")
            unit = input("unit: ")

            print(action, distance, unit)


if __name__ == "__main__":
    try:
        controller_node = Node(node_name = "controller_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
