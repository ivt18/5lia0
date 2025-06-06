#!/usr/bin/env python2

import rospy
from motorDriver import *

from controller_package.msg import MovementRequest
from motor_driver_package.msg import MotorSpeedRequest

class MotorDriverNode:
    
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        self.motor = DaguWheelsDriver()

        rospy.loginfo("Initializing motor driver node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscriber
        self.subscriber = rospy.Subscriber(
            "/motor_driver/motors",
            MotorSpeedRequest,
            self.receiveRequest,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10,
        )

        self.initialized = True
        rospy.loginfo("Motor driver node initialized!")

    def receiveRequest(self, motor_data):
        if not self.initialized:
            return

        self.motor.set_wheel_speed(
            left = motor_data.speed_left_wheel,
            right = motor_data.speed_right_wheel
        )
        rospy.loginfo("Received request: left_wheel = %f, right_wheel = %f",
            motor_data.speed_left_wheel, motor_data.speed_right_wheel)


if __name__ == "__main__":
    try:
        motor_driver_node = MotorDriverNode(node_name = "motor_driver_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
