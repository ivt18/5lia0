#!/usr/bin/env python2

import rospy

import datatypes


class MotionPlanningNode:
    
    def __init__(self, name):
        self.initialized = False
        self.name = name
        rospy.loginfo("Initializing {name}...".format(name=self.name))
        rospy.init_node(self.name, anonymous=True)

        # car status
        self.position = datatypes.Position()
        self.safety_car_position = datatypes.SafetyCarPosition()

        # Construct subscribers
        self.position = rospy.Subscriber(
            "motor-control/position",
            datatypes.Position,
            self.read_pos,
            buff_size=10,
            queue_size=1,
        )

        self.camera = rospy.Subscriber(
            "camera/fusion",
            Float64,
            buff_size=10,
            queue_size=1,
        )

        self.lidar = rospy.Subscriber(
            "/tof/distance",
            UInt16,
            self.read_tof,
            buff_size=10,
            queue_size=1,
        )

        # Construct publisher
        self.publisher = rospy.Publisher(
            "motor-control/motion-planning",
            datatypes.MovementRequest,
            queue_size=1,
            self.publish(),
        )

        self.initialized = True
        rospy.loginfo("{name} initialized".format(name=self.name))
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish)
    
    
    def publish(self, event):
        pass

