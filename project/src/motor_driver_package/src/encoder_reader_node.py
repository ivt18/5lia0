#!/usr/bin/env python2

import math

import numpy as np
import rospy

from config import get_wheel_encoder_config, get_car_config, get_tickrate
from encoderDriver import WheelEncoderDriver, WheelDirection
from motor_driver_package.msg import EncoderData, MotorSpeedRequest
from datatypes import CarConfig


class EncoderReaderNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {node}...".format(node=node_name))
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscriber
        self.motors_topic = rospy.Subscriber(
            "/motor_driver/motors",
            MotorSpeedRequest,
            self.read_motors,
            buff_size=1000000,
            queue_size=1,
        )

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/encoder",
            EncoderData,
            queue_size=1,
        )

        # encoder drivers
        config = get_wheel_encoder_config()
        self.driver_left = WheelEncoderDriver(config["GPIO_LEFT"])
        self.driver_right = WheelEncoderDriver(config["GPIO_RIGHT"])

        # car config
        self.config = get_car_config()

        # number of ticks per full wheel rotation
        self.resolution = config["resolution"]
        # last recorded value of ticks for left/right wheel
        self.ticks = (0, 0)
        self.tickrate = get_tickrate()

        self.initialized = True
        rospy.loginfo("{node} initialized".format(node=node_name))
        self.timer = rospy.Timer(rospy.Duration(self.tickrate), self.publish)
    
    def read_motors(self, data):
        if not self.initialized:
            return
        
        # update wheel direction (moving forward/reverse)
        direction_left = WheelDirection.FORWARD if data.speed_left_wheel > 0 else WheelDirection.REVERSE
        direction_right = WheelDirection.FORWARD if data.speed_right_wheel > 0 else WheelDirection.REVERSE

        self.driver_left.set_direction(direction_left)
        self.driver_right.set_direction(direction_right)

        rospy.logdebug("Set left wheel direction to {left}\n"
                        "Set right wheel direction to {right}"
                       .format(left="FORWARD" if direction_left == WheelDirection.FORWARD else "REVERSE",
                               right="FORWARD" if direction_left == WheelDirection.FORWARD else "REVERSE")
        )

    def publish(self, event):
        msg = EncoderData()

        # compute tick delta left wheel
        ticks_left = self.driver_left._ticks
        d_left = self.delta_phi(ticks_left, 0)
        msg.delta_left = d_left

        # compute tick delta right wheel
        ticks_right = self.driver_right._ticks
        d_right = self.delta_phi(ticks_right, 1)
        msg.delta_right = d_right

        # compute speed left wheel
        distance_travelled_left = float(self.config.wheel_radius) * float(d_left)
        msg.v_left = float(distance_travelled_left) / float(self.tickrate)

        # compute speed right wheel
        distance_travelled_right = float(self.config.wheel_radius) * float(d_right)
        msg.v_right = float(distance_travelled_right) / float(self.tickrate)

        # publish message
        self.publisher.publish(msg)
        # rospy.loginfo("Published encoder data [{delta_left} {delta_right}]".format(delta_left=d_left, delta_right=d_right))

        # update ticks
        self.ticks = (ticks_left, ticks_right)

    def delta_phi(self, ticks, direction):
        """
        Args:
            ticks: Current tick count from the encoders.
            direction: 1 right; 0 left
        Return:
            dphi: Rotation of the wheel in radians.
        """

        delta_ticks = ticks - self.ticks[direction]

        delta_rot = float(delta_ticks) / float(self.resolution)
        dphi = delta_rot * 2 * np.pi

        return dphi


if __name__ == '__main__':
    try:
        encoder_reader_node = EncoderReaderNode("encoder_reader_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
