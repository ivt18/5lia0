#!/usr/bin/env python2

import numpy as np
import rospy

from config import get_car_config
from datatypes import Position
from motor_driver_package.msg import Position, EncoderData


class PositionEstimatorNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {name}...".format(name=node_name))
        rospy.init_node(self.node_name, anonymous=True)

        self.config = get_car_config()
        self.position = Position(0, 0, 0)

        # Construct subscriber
        self.encoder = rospy.Subscriber(
            "/motor_driver/encoder",
            EncoderData,
            self.read_encoder,
            buff_size=1000000,
            queue_size=1,
        )

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/position",
            Position,
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("{name} initialized".format(name=node_name))

    
    def read_encoder(self, encoder_data):
        if not self.initialized:
            return
        
        # update position
        self.position = self.position_estimation_relative(
            self.position,
            self.config.wheel_radius,
            self.config.wheelbase,
            encoder_data.delta_left, encoder_data.delta_right)

        # publish updated position
        self.publish()

    
    def publish(self):
        msg = Position()
        msg.x = self.position.x
        msg.y = self.position.y
        msg.theta = self.position.theta

        self.publisher.publish(msg)


    def position_estimation_relative(self,
        relative_position, R, wheelbase,
        delta_phi_left, delta_phi_right):
        """
        Calculate the current Duckiebot position using the dead-reckoning model.

        Args:
            relative_position:  base position to compute estimation with respect to
            R:                  wheel radius (assume both wheels are of the same radius)
            wheelbase:          vehicle wheelbase
            delta_phi_left:     left wheel rotation (rad)
            delta_phi_right:    right wheel rotation (rad)

        Return:
            estimation:         Position object containing estimated x, y and heading
        """

        estimation = Position(0, 0, 0)

        d_left = R * delta_phi_left
        d_right = R * delta_phi_right
        delta_theta = (d_right - d_left) / wheelbase
        estimation.theta = relative_position.theta + delta_theta

        delta_dist = (d_left + d_right) / 2
        delta_x = delta_dist * np.cos(estimation.theta)
        delta_y = delta_dist * np.sin(estimation.theta)

        estimation.x = relative_position.x + delta_x
        estimation.y = relative_position.y + delta_y

        return estimation


if __name__ == '__main__':
    try:
        position_estimator = PositionEstimatorNode(node_name = "position_estimator_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
