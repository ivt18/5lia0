#!/usr/bin/env python2

import numpy as np
import rospy

from motor_driver_package.msg import MotorSpeedRequest, MovementRequest

from config import get_car_config, get_motor_calibration_config
import datatypes

PROPORTIONAL_GAIN = 5

class ReceiverNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        rospy.loginfo("Initializing receiver node...")
        rospy.init_node(self.node_name, anonymous=True)

        self.config = get_car_config()
        self.motor_calibration = get_motor_calibration_config()

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/motors",
            MotorSpeedRequest,
            queue_size=10
        )

        # Construct subscribers
        self.commands_subscriber = rospy.Subscriber(
            "/motor_control/motion_planning",
            MovementRequest,
            self.receiveRequest,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10
        )

        self.initialized = True
        rospy.loginfo("Receiver node initialized!")


    def receiveRequest(self, data):
        if not self.initialized:
            return

        rospy.loginfo("RECEIVED A REQUEST!!!!")

        mult_left = float(data.v) + self.config.wheelbase * PROPORTIONAL_GAIN * float(data.angle)
        mult_right = float(data.v) - self.config.wheelbase * PROPORTIONAL_GAIN * float(data.angle)
        left = float(mult_left) * (float(self.motor_calibration["gain"]) - float(self.motor_calibration["trim"]))
        right = float(mult_right) * (float(self.motor_calibration["gain"]) + float(self.motor_calibration["trim"]))

        motor_request = MotorSpeedRequest()
        motor_request.speed_left_wheel = left
        motor_request.speed_right_wheel = right
        self.publisher.publish(motor_request)


if __name__ == "__main__":
    try:
        receiver_node = ReceiverNode(node_name = "receiver_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
