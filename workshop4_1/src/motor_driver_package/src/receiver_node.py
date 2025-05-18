#!/usr/bin/env python2

import rospy
import numpy

from controller_package.msg import MovementRequest
from motor_driver_package.msg import MotorSpeedRequest, Position

from config import get_car_config
from datatypes import MovementRequest, Position

from position_estimator_node import position_estimator_relative

class ReceiverNode:
    
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        rospy.loginfo("Initializing receiver node...")
        rospy.init_node(self.node_name, anonymous=True)

        self.config = get_car_config()
        self.motor_calibration = get_motor_calibration_config()
        self.current_position = Position(0, 0, 0)

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/motors",
            MotorSpeedRequest,
            queue_size=10,
        )

        # Construct subscribers
        self.commands_subscriber = rospy.Subscriber(
            "/motor_driver/commands",
            MovementRequest,
            self.receiveRequest,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10,
        )

        self.position_subscriber = rospy.Subscriber(
            "/motor_driver/position",
            Position,
            self.receivePosition,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10,
        )

        self.initialized = True
        rospy.loginfo("Receiver node initialized!")


    # get the distance from the robot's current position to a target position
    def getDistanceTo(self, position):
        start_vector = np.array([self.current_position.x, self.current_position.y])
        target_vector = np.array([position.x, position.y])
        return np.linalg_norm(target_vector - start_vector)


    def receivePosition(self, data):
        if not self.initialized:
            return

        self.current_position.x = data.x
        self.current_position.y = data.y
        self.current_position.theta = data.theta


    def receiveRequest(self, data):
        if not self.initialized:
            return

        request = MovementRequest(data.request_type, data.value)
        target_position = Position(0, 0, 0)
        mult_left = 1
        mult_right = 1
        
        if (request.request_type = MovementRequest.MOVEMENT_REQUEST):
            delta_req = data.value / self.config.wheel_radius # radians
            target_position = position_estimator_relative(
                self.current_position,
                self.config.wheel_radius,
                self.config.wheelbase,
                delta_req, delta_req)
        
        if (request.request_type = MovementRequest.TURN_REQUEST):
            delta_left = 0 # radians
            delta_right = 0 # radians
            distance = request.value * np.pi * self.config.wheelbase # meters, arch length

            if (distance < 0):
                mult_left = 0
                delta_right = -(distance / self.config.wheel_radius)
            else:
                mult_right = 0
                delta_left = distance / self.config.wheel_radius

            target_position = position_estimator_relative(
                self.current_position,
                self.config.wheel_radius,
                self.config.wheelbase,
                delta_left, delta_right)

        self.motor.set_wheel_speed(
            left = mult_left * (self.motor_calibration.gain - self.motor_calibration.trim),
            right = mult_right * (self.motor_calibration.gain + self.motor_calibration.trim))

        while (self.getDistanceTo(target_position) > 0.05): # 5 cm tolerance (might be too much)
            rospy.Rate(10).sleep

        self.motor.set_wheel_speed(left = 0, right = 0)


if __name__ == "__main__":
    try:
        motor_driver_node = MotorDriverNode(node_name = "receiver_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
