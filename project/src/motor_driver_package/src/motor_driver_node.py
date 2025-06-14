#!/usr/bin/env python2

import rospy
from motorDriver import *

from config import get_tickrate
from controller_package.msg import MovementRequest
from motor_driver_package.msg import EncoderData, MotorSpeedRequest

# TODO: define proportional gains in the config
PROPORTIONAL_GAIN = 0.35


class MotorDriverNode:
    
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        self.motor = DaguWheelsDriver()

        rospy.loginfo("Initializing motor driver node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/motor_driver/motors",
            MotorSpeedRequest,
            self.receiveRequest,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10,
        )

        self.velocity_subscriber = rospy.Subscriber(
            "/motor_driver/encoder",
            EncoderData,
            self.receiveVelocity,
            buff_size=1000000,
            queue_size=10,
        )

        self.tickrate = get_tickrate()

        # the velocity at which the car is moving
        self.v_left = 0.0
        self.v_right = 0.0

        # the target velocity of the car
        self.target_v_left = 0.0
        self.target_v_right = 0.0

        # the signals being sent to the motors
        self.m_left = 0.0
        self.m_right = 0.0
        self.timer = rospy.Timer(rospy.Duration(self.tickrate), self.updateVelocity)

        self.initialized = True
        rospy.loginfo("Motor driver node initialized!")

    
    def receiveVelocity(self, velocity_data):
        self.v_left = velocity_data.v_left
        self.v_right = velocity_data.v_right


    # TODO: apply the trim here
    def updateVelocity(self, event):
        if not self.initialized:
            return

        diff_left = float(self.target_v_left - self.v_left)
        diff_right = float(self.target_v_right - self.v_right)
        self.m_left += diff_left * PROPORTIONAL_GAIN
        self.m_right += diff_right * PROPORTIONAL_GAIN

        self.motor.set_wheel_speed(
            left = self.m_left,
            right = self.m_right
        )


    def receiveRequest(self, motor_data):
        if not self.initialized:
            return

        self.target_v_left = motor_data.speed_left_wheel
        self.target_v_right = motor_data.speed_right_wheel

        # rospy.loginfo("Received request: left_wheel = %f, right_wheel = %f", motor_data.speed_left_wheel, motor_data.speed_right_wheel)


if __name__ == "__main__":
    try:
        motor_driver_node = MotorDriverNode(node_name = "motor_driver_node")
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
    # except KeyboardInterrupt:
        # motor_driver_node.motor.set_wheel_speed(left=0, right=0)

