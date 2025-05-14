#!/usr/bin/env python2

import numpy as np
import rospy

from config.config import get_car_config
from msg import Pose, EncoderData

from datatypes import Pose


class PoseEstimatorNode:

    def __init__(self, node_name: str):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {name}...".format(name=node_name))
        rospy.init_node(self.node_name, anonymous=True)

        self.config = get_car_config()
        self.pose = Pose(0, 0, 0)

        # Construct subscriber
        self.encoder = rospy.Subscriber(
            "/motor_driver/encoder",
            EncoderData,
            self.read_encoder,
            buff_size=1000000,
            queue_size=1,
        )

        # Construct pulisher
        self.publisher = rospy.Publisher(
            "/motor_driver/pose",
            Pose,
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("{name} initialized".format(node_name))
    
    def read_encoder(self, encoder_data):
        if not self.initialized:
            return
        
        # update pose
        self.pose = self.pose_estimation(encoder_data.delta_left.data, encoder_data.delta_right.data)

        # publish updated pose
        self.publish()
    
    def publish(self, event):
        msg = Pose()
        msg.x.data = self.pose.x
        msg.y.data = self.pose.y
        msg.theta.data = self.pose.theta

        self.publisher.publish(msg)
        rospy.loginfo("Published current pose estimation")
    
    def pose_estimation(self, delta_phi_left, delta_phi_right):

        """
        Calculate the current Duckiebot pose using the dead-reckoning model.

        Args:
            delta_phi_left:     left wheel rotation (rad)
            delta_phi_right:    right wheel rotation (rad)

        Return:
            estimation:         Pose object containing estimated x, y and heading
        """

        # radius of the wheel (both wheels are assumed to have the same size)
        R = self.wheel_radius

        estimation = self.pose

        d_left = R * delta_phi_left
        d_right = R * delta_phi_right
        delta_theta = (d_right - d_left) / self.wheelbase
        estimation.theta = self.pose.theta + delta_theta

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
