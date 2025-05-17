#!/usr/bin/env python3

import time

import numpy as np
import rospy
from controller_package.msg import MovementRequest
from motor_driver_package.msg import Pose


class ControllerNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing controller node...")
        rospy.init_node(self.node_name, anonymous=True)
        self.wheel_radius = 0.032
        self.wheel_base = 0.17
        self.resolution = 137

        self.current_pose = Pose()
        self.current_pose.x = 0
        self.current_pose.y = 0
        self.current_pose.theta = 0

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/motor_driver/motors",
            MovementRequest,
            queue_size=10,
        )

        self.subscriber = rospy.Subscriber(
            "/motor_driver/poser",
            Pose,
            self.update_pose,
            queue_size=10,
        )

        rospy.loginfo("Controller node initialized!")

        # while True:
        # action = input("desired action: ")
        # distance = input("distance: ")
        # unit = input("unit: ")

        # print(action, distance, unit)

        #   msg = MovementRequest()

        #    msg.left_wheel = int(input("left: "))
        #    msg.right_wheel = int(input("right: "))
        #    self.publisher.publish(msg)
        #    rospy.loginfo(
        #        "Sent request: left_wheel = %s, right_wheel = %s",
        #        msg.left_wheel,
        #        msg.right_wheel,
        #

    # receive current pose from encoder
    def update_pose(self, msg):
        rospy.loginfo("Now at %s, %s, %s", msg.x, msg.y, msg.theta)
        if not self.initialized:
            self.initialized = True
        self.current_pose.x = msg.x
        self.current_pose.y = msg.y
        self.current_pose.theta = msg.theta

    # computes distance between two Poses
    def get_distance(self, start, end):
        start_v = np.array([start.x, start.y])
        end_v = np.array([end.x, end.y])
        return np.linalg.norm(end_v - start_v)

    # moves straight a certain distance at a constant speed
    # assumes distance is in m, speed is in pwm (-1:1)
    # no endometry feedback control (yet), assumes destination is always reached
    def move_straight(self, distance, speed):
        if self.initialized:
            start = self.current_pose
            dest = self.get_dest(distance, 0)
            start_msg = MovementRequest()
            start_msg.left_wheel = int(speed)
            start_msg.right_wheel = int(speed)
            self.publisher.publish(start_msg)
            rospy.loginfo("Start moving at %s pwm", speed)

            while self.get_distance(start, self.current_pose) < distance:
                rospy.Rate(10).sleep

            stop_msg = MovementRequest()
            stop_msg.left_wheel = 0
            stop_msg.right_wheel = 0
            self.publisher.publish(stop_msg)
            rospy.loginfo("Stopped moving after %s m", distance)

    # turns at an angle at constant speed
    # assumes angle is in rad
    def turn(self, angle, speed):
        if self.initialized:
            dest_angle = self.current_pose.theta + angle
            start_msg = MovementRequest()
            if angle > 0:
                start_msg.left_wheel = -int(speed)
                start_msg.right_wheel = int(speed)
            elif angle < 0:
                start_msg.left_wheel = int(speed)
                start_msg.right_wheel = -int(speed)
            else:
                start_msg.left_wheel = 0
                start_msg.right_wheel = 0
            self.publisher.publish(start_msg)
            rospy.loginfo("Start turning at %s pwm", speed)

            while self.current_pose.theta != dest_angle:
                rospy.Rate(10).sleep

            stop_msg = MovementRequest()
            stop_msg.left_wheel = 0
            stop_msg.right_wheel = 0
            self.publisher.publish("Stopped turning after %s rad", angle)


if __name__ == "__main__":
    try:
        controller_node = ControllerNode("controller_node")
        controller_node.move_straight(1, 1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
