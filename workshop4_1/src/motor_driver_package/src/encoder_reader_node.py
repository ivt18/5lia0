#!/usr/bin/env python2

import numpy as np
import rospy

from config import get_wheel_encoder_config
from encoderDriver import WheelEncoderDriver, WheelDirection
from motor_driver_package.msg import EncoderData, PIDData


class EncoderReaderNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {node}...".format(node=node_name))
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscriber
        self.motors_topic = rospy.Subscriber(
            "/motor_driver/motors",
            PIDData,
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

        # number of ticks per full wheel rotation
        self.resolution = config["resolution"]
        # last recorded value of ticks for left/right wheel
        self.ticks = (0, 0)

        self.initialized = True
        rospy.loginfo("{node} initialized".format(node=node_name))
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish)
    
    def read_motors(self, data):
        if not self.initialized:
            return
        
        # update wheel direction (moving forward/reverse)
        direction_left = WheelDirection.FORWARD if data.direction_left.data else WheelDirection.REVERSE
        direction_right = WheelDirection.FORWARD if data.direction_right.data else WheelDirection.REVERSE

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

        # publish message
        self.publisher.publish(msg)
        rospy.loginfo("Published encoder data [{delta_left} {delta_right}]".format(delta_left=d_left, delta_right=d_right))

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

        delta_rot = delta_ticks / self.resolution
        dphi = delta_rot * 2 * np.pi

        return dphi


if __name__ == '__main__':
    try:
        encoder_reader_node = EncoderReaderNode("encoder_reader_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
