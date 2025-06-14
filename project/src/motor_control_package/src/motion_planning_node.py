#!/usr/bin/env python2

import math

import rospy
from std_msgs.msg import UInt16, Float64

import datatypes

from motor_driver_package.msg import EncoderData, Position, MovementRequest
from config import get_tickrate


TARGET_DISTANCE = 0.15  # desired distance to safety car
TIME_TO_TARGET = 1      # desired time to reach the target position in seconds


class MotionPlanningNode:
    
    def __init__(self, name):
        self.initialized = False
        self.name = name
        rospy.loginfo("Initializing {name}...".format(name=self.name))
        rospy.init_node(self.name, anonymous=True)

        # car status
        self.position = datatypes.Position()
        self.speed = 0
        self.distance_travelled = 0     # distance travelled since last update
        self.safety_car_position = datatypes.SafetyCarPosition(d=TARGET_DISTANCE)

        # config
        self.tickrate = get_tickrate()

        # Construct subscribers
        self.position = rospy.Subscriber(
            "motor_control/position",
            Position,
            self.read_pos,
            buff_size=10,
            queue_size=1,
        )

        self.encoder = rospy.Subscriber(
            "motor_driver/encoder",
            EncoderData,
            self.read_encoder,
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
        self.timer = rospy.Timer(rospy.Duration(self.tickrate), self.publish)
    

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


    def read_encoder(self, data):
        """
        Calculate current ground speed from the rotational speed of the wheels
        """
        speed_left = float(data.v_left)
        speed_right = float(data.v_right)

        self.speed = float(speed_left + speed_right) / float(2)


    def read_camera(self, data):
        """
        Update the angle to the safety car as seen by the camera
        """
        self.safety_car_position.angle = data.data


    def read_tof(self, data):
        """
        Update distance to the safety car as measured by the LiDAR
        """
        # rospy.loginfo("received TOF: {d}".format(d=data.data))
        # past 1.2m the reading is no longer stable and cannot be trusted
        if data.data <= 1200:
            self.safety_car_position.distance = float(data.data) / 1000

    
    def publish(self, event):
        """
        Calculate and publish the required linear and angular velocities to keep up with the safety car
        """
        msg = MovementRequest()

        # linear velocity
        """
        distance_error = self.safety_car_position.distance - TARGET_DISTANCE            # difference between target distance to SC and actual distance
        coeff = 1 if self.safety_car_position.distance >= TARGET_DISTANCE else -1       # if distance has increased, v_SC has increased, else v_SC has decreased
        distance_sc = self.distance_travelled + coeff * (self.safety_car_position.distance - TARGET_DISTANCE)   # distance travelled by the SC since last tick     
        d = distance_error + distance_sc                                                # distance that needs to be travelled until the next tick
        msg.v = d / self.tickrate
        """
        distance_error = self.safety_car_position.distance - TARGET_DISTANCE            # difference between target distance to SC and actual distance
        speed_adjustment = distance_error / (5 * self.tickrate)                               # m/s
        total_speed = self.speed + speed_adjustment
        msg.v = 0.5
        
        # angle to the safety car
        msg.angle = self.safety_car_position.angle    # rad

        # rospy.loginfo("d_SC: {dsc};\td_T: {dt}\td_e: {de}".format(dsc=self.safety_car_position.distance, dt=TARGET_DISTANCE, de=distance_error))
        rospy.loginfo("v: {v_now}\ttarget v: {v};\ttarget angle: {theta}".format(v_now=self.speed, v=msg.v, theta=msg.angle))
        self.publisher.publish(msg)


if __name__ == '__main__':
    mp_node = MotionPlanningNode("motion_planning_node")
    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)

