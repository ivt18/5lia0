#!/usr/bin/env python3

import time

import rospy
from controller_package.msg import MovementRequest


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
            # Change buff size and queue size accordingly
            queue_size=10,
        )

        self.sendSequence = 0

        self.initialized = True
        rospy.loginfo("Controller node initialized!")

        while True:
            # action = input("desired action: ")
            # distance = input("distance: ")
            # unit = input("unit: ")

            # print(action, distance, unit)

            msg = MovementRequest()

            msg.left_wheel = int(input("left wheel speed: "))
            msg.right_wheel = int(input("right wheel speed: "))
            self.publisher.publish(msg)
            rospy.loginfo(
                "Sent request: left_wheel = %s, right_wheel = %s",
                msg.left_wheel,
                msg.right_wheel,
            )


if __name__ == "__main__":
    try:
        controller_node = ControllerNode("controller_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
