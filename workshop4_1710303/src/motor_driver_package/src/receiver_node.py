#!/usr/bin/env python2

import numpy as np
import rospy

from controller_package.msg import MovementRequest
from motor_driver_package.msg import MotorSpeedRequest, Position

from config import get_car_config, get_motor_calibration_config
#from datatypes import MovementRequest, Position
import datatypes

class ReceiverNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        rospy.loginfo("Initializing receiver node...")
        rospy.init_node(self.node_name, anonymous=True)

        self.config = get_car_config()
        self.motor_calibration = get_motor_calibration_config()
        self.current_position = datatypes.Position(0, 0, 0)

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/motors",
            MotorSpeedRequest,
            queue_size=10
        )

        # Construct subscribers
        self.commands_subscriber = rospy.Subscriber(
            "/motor_driver/commands",
            MovementRequest,
            self.receiveRequest,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10
        )

        self.position_subscriber = rospy.Subscriber(
            "/motor_driver/position",
            Position,
            self.receivePosition,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=10
        )

        self.initialized = True
        rospy.loginfo("Receiver node initialized!")


    # get the distance from the robot's current position to a target position
    def getDistanceTo(self, position):
        start_vector = np.array([self.current_position.x, self.current_position.y])
        target_vector = np.array([position.x, position.y])
        return np.linalg.norm(target_vector - start_vector)


    def getRotationTo(self, position):
        return position.theta - self.current_position.theta;


    def receivePosition(self, data):
        if not self.initialized:
            return

        self.current_position.x = data.x
        self.current_position.y = data.y
        self.current_position.theta = data.theta


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


    def receiveRequest(self, data):
        if not self.initialized:
            return

        rospy.loginfo("Received request of type {} with value {}".format(data.request_type, data.value))

        request = datatypes.MovementRequest(data.request_type, data.value)
        target_position = datatypes.Position(0, 0, 0)
        mult_left = 1
        mult_right = 1
        
        if (request.request_type == datatypes.MovementRequest.MOVEMENT_REQUEST):
            delta_req = data.value / self.config.wheel_radius # radians
            target_position = self.position_estimation_relative(
                self.current_position,
                self.config.wheel_radius,
                self.config.wheelbase,
                delta_req, delta_req)
        
        if (request.request_type == datatypes.MovementRequest.TURN_REQUEST):
            delta_left = 0 # radians
            delta_right = 0 # radians
            distance = request.value * self.config.wheelbase # meters, arch length

            # for some reason these deltas should have their signs flipped
            # TODO: potential bug with position estimator
            if (distance < 0):
                mult_left = 0
                delta_right = -(distance / self.config.wheel_radius)
            else:
                mult_right = 0
                delta_left = distance / self.config.wheel_radius

            target_position = self.position_estimation_relative(
                self.current_position,
                self.config.wheel_radius,
                self.config.wheelbase,
                delta_left, delta_right)

        left = float(mult_left) * (float(self.motor_calibration["gain"]) - float(self.motor_calibration["trim"]))
        right = float(mult_right) * (float(self.motor_calibration["gain"]) + float(self.motor_calibration["trim"]))

        motor_request = MotorSpeedRequest()
        motor_request.speed_left_wheel = left
        motor_request.speed_right_wheel = right
        self.publisher.publish(motor_request)

        if (request.request_type == datatypes.MovementRequest.MOVEMENT_REQUEST):
            prev_distance = self.getDistanceTo(target_position)
            current_distance = prev_distance
            while (current_distance > 0.03 and current_distance < prev_distance + 0.01): # 3 cm tolerance
                rospy.loginfo("%f %f", self.current_position.x, self.current_position.y)
                prev_distance = current_distance
                current_distance = self.getDistanceTo(target_position)
                rospy.Rate(20).sleep

        if (request.request_type == datatypes.MovementRequest.TURN_REQUEST):
            prev_rotation = np.abs(self.getRotationTo(target_position))
            current_rotation = prev_rotation
            while (current_rotation > 0.03 and current_rotation < prev_rotation + 0.01): # 0.1 rad tolerance
                rospy.loginfo("%f", self.current_position.theta)
                prev_rotation = current_rotation
                current_rotation = np.abs(self.getRotationTo(target_position))
                rospy.Rate(20).sleep

        rospy.loginfo("Arrived at target destination")

        stop_request = MotorSpeedRequest()
        stop_request.speed_left_wheel = 0
        stop_request.speed_right_wheel = 0
        self.publisher.publish(stop_request)


if __name__ == "__main__":
    try:
        receiver_node = ReceiverNode(node_name = "receiver_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
