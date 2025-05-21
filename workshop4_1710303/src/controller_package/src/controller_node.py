#!/usr/bin/env python2

import math
import time

import numpy as np
import rospy
from controller_package.msg import MovementRequest
from jetson_camera.msg import ObjectPosition


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

        self.subscriber = rospy.subscriber(
            "/camera/object_position", ObjectPosition, self.track_object, queue_size=1
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

    def track_object(self, object_position):
        kp = 0.1
        center_x = object_position.image_width / 2
        object_x = object_position.x + object_position.width / 2
        error_x = object_x - center_x

        if error_x < 30:
            msg_move = MovementRequest()
            msg_move.request_type = 0
            msg_move.value = 0.5
            self.publisher.publish(msg_move)
        else:

            angle = kp * error_x
            angle = max(min(angle, 30), -30)

            msg_turn = MovementRequest()
            msg_turn.request_type = 1
            msg_turn.value = 2 * math.pi * angle / 360
            self.publisher.publish(msg_turn)
        return


if __name__ == "__main__":
    try:
        controller_node = ControllerNode("controller_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
