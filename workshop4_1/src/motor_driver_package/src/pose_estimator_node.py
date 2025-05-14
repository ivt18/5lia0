#!/usr/bin/env python2

from typing import Tuple

import numpy as np
import rospy

from config.config import get_car_config
from msg import Pose

from datatypes import Pose, CarConfig


class PoseEstimatorNode:

    def __init__(self, node_name: str):
        self.initialized: bool = False
        self.node_name: str = node_name
        rospy.loginfo("Initializing {name}...".format(name=node_name))
        rospy.init_node(self.node_name, anonymous=True)

        self.config: CarConfig = get_car_config()
        self.pose: Pose = Pose(0, 0, 0)

        # Construct subscriber
        self.encoder: rospy.Subscriber = rospy.Subscriber(
            "/encoder",
            Encoder,
            self.read_encoder,
            buff_size=1000000,
            queue_size=1,
        )

        # Construct pulisher
        self.publisher: rospy.Publisher = rospy.Publisher(
            "/motor_driver/pos",
            Pose,
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("{name} initialized".format(node_name))
        # TODO: set publish timer
    
    def read_encoder(self, data) -> None:
        if not self.initialized:
            return
        
        # TODO: read encoder data
        left_theta
        right_theta

        # TODO: use delta_phi? how?
        
        self.pose = self.pose_estimation(
            self.pose.theta,
            left_theta,
            right_theta
        )
    
    def publish(self, event) -> None:
        # TODO: do we publish periodically or whenever we get new encoder data?
        
        msg = Pose()
        msg.x.data = self.pose.x
        msg.y.data = self.pose.y
        msg.theta.data = self.pose.theta

        self.publisher.publish(msg)
        rospy.loginfo("Published current pose estimation")
    
    def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
        """
        Args:
            ticks: Current tick count from the encoders.
            prev_ticks: Previous tick count from the encoders.
            resolution: Number of ticks per full wheel rotation returned by the encoder.
        Return:
            dphi: Rotation of the wheel in radians.
            ticks: current number of ticks.
        """

        delta_ticks = ticks - prev_ticks
        N_tot = 135

        delta_rot = delta_ticks / N_tot
        dphi = delta_rot * 2 * np.pi

        return dphi, ticks
    
    def pose_estimation(self,
        theta_prev: float,
        delta_phi_left: float,
        delta_phi_right: float,
    ) -> Tuple[float, float, float]:

        """
        Calculate the current Duckiebot pose using the dead-reckoning model.

        Args:
            x_prev:             previous x estimate - assume given
            y_prev:             previous y estimate - assume given
            theta_prev:         previous orientation estimate - assume given
            delta_phi_left:     left wheel rotation (rad)
            delta_phi_right:    right wheel rotation (rad)

        Return:
            x_curr:                  estimated x coordinate
            y_curr:                  estimated y coordinate
            theta_curr:              estimated heading
        """

        # radius of the wheel (both wheels are assumed to have the same size)
        R: float = self.wheel_radius
        # distance from wheel to wheel
        wheelbase: float = self.wheelbase

        estimation: Pose = self.pose

        d_left = R * delta_phi_left
        d_right = R * delta_phi_right
        delta_theta = (d_right - d_left) / wheelbase
        estimation.theta = theta_prev + delta_theta

        delta_dist = (d_left + d_right) / 2
        delta_x = delta_dist * np.cos(estimation.theta)
        delta_y = delta_dist * np.sin(estimation.theta)

        estimation.x = self.pose.x + delta_x
        estimation.y = self.pose.y + delta_y

        return estimation


if __name__ == '__main__':
    try:
        pose_estimator = PoseEstimatorNode(node_name = "pose_estimator_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
