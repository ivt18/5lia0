#!/usr/bin/env python2

import math

import rospy
from std_msgs.msg import UInt16, Float64

import datatypes

from motor_driver_package.msg import Position, MovementRequest


TARGET_DISTANCE = 0.15  # desired distance to safety car
TIME_TO_TARGET = 1      # desired time to reach the target position in seconds
TICK_RATE = 0.05        # refresh rate of publisher


class MotionPlanningNode:
    
    def __init__(self, name):
        self.initialized = False
        self.name = name
        rospy.loginfo("Initializing {name}...".format(name=self.name))
        rospy.init_node(self.name, anonymous=True)

        # car status
        self.position = datatypes.Position()
        self.distance_travelled = 0     # distance travelled since last update
        self.safety_car_position = datatypes.SafetyCarPosition()

        # Construct subscribers
        self.position = rospy.Subscriber(
            "motor_control/position",
            Position,
            self.read_pos,
            buff_size=10,
            queue_size=1,
        )

        self.camera = rospy.Subscriber(
            "camera/fusion",
            Float64,
            self.read_camera,
            buff_size=10,
            queue_size=1,
        )

        self.lidar = rospy.Subscriber(
            "/motor_control/tof",
            UInt16,
            self.read_tof,
            buff_size=10,
            queue_size=1,
        )

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_control/motion_planning",
            MovementRequest,
            queue_size=1,
        )

        self.initialized = True
        rospy.loginfo("{name} initialized".format(name=self.name))
        self.timer = rospy.Timer(rospy.Duration(TICK_RATE), self.publish)
    

    def read_pos(self, data):
        """
        Update current position of the car
        """
        # update distance travelled
        prev = [self.position.x, self.position.y]   # coordinates at previous update
        now = [data.x, data.y]                      # coordinates now
        self.distance_travelled = math.dist(prev, now)

        # update current position
        self.position.x = data.x
        self.position.y = data.y
        self.position.theta = data.theta


    def read_camera(self, data):
        """
        Update the angle to the safety car as seen by the camera
        """
        self.safety_car_position.angle = data.data
        rospy.loginfo("Safety car angle: {angle}".format(angle=self.safety_car_position.angle))


    def read_tof(self, data):
        """
        Update distance to the safety car as measured by the LiDAR
        """
        self.safety_car.distance = data.distance

    
    def publish(self):
        """
        Calculate and publish the required linear and angular velocities to keep up with the safety car
        """
        msg = MovementRequest()

        # linear velocity
        coeff = 1 if self.safety_car.distance >= TARGET_DISTANCE else -1    # if distance has increased, v_SC has increased, else v_SC has decreased
        sc_speed = (self.distance_travelled + coeff * self.safety_car.distance) / TICK_RATE     # m/s
        msg.v = sc_speed
        
        # angle to the safety car
        msg.theta = safety_car.angle    # rad

        rospy.loginfo("target v: {v};\ttarget angle: {theta}".format(v=msg.v, theta=msg.theta))
        self.publisher.publish(msg)


if __name__ == '__main__':
    mp_node = MotionPlanningNode("motion_planning_node")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

