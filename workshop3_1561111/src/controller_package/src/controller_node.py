#!/usr/bin/env python2

import time

import numpy as np
import rospy
from controller_package.msg import MovementRequest
from jetson_camera.msg import QRCodes


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

        # Construct subscriber
        self.sub_qr_code = rospy.Subscriber(
            "/camera/qr_code",
            QRCodes,
            self.qr_code_cb,
            queue_size=10
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

    def qr_code_cb(self, data):
        if not self.initialized:
            return

        # Process the QR code data
        if data.found:
            msg = MovementRequest()
            msg.request_type = 1
            msg.value = 1
            self.publisher.publish(msg)

            rospy.loginfo("Found QR code: %s", data.data)

            # Here you can add logic to handle the QR code data
            # For example, you can send a command to the motors based on the QR code data


if __name__ == "__main__":
    try:
        controller_node = ControllerNode("controller_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
