#!/usr/bin/env python2

import rospy

from std_msgs.msg import UInt16

from tofDriver import VL53L0X


N = 10      # number of values to record before sending a moving average


class TofNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {name}...".format(name=self.node_name))
        rospy.init_node(self.node_name, anonymous=True)

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/tof/distance",
            UInt16,
            queue_size=1,
        )

        # ToF sensor driver
        self.sensor = VL53L0X()

        self.distances = []     # list of recorded values
        self.cycle_idx = 0      # current index of circular buffer to update self.distances

        self.initialized = True
        rospy.loginfo("Node {name} initialized!".format(name=self.node_name))
        self.timer = rospy.Timer(rospy.Duration(0.1), self.read_sensor)

    def publish(self):
        # check if at least N values have been recorded
        if len(self.distances) < N:
            return

        # compute moving average
        msg = UInt16()
        msg.data = sum(self.distances) / N

        # send message
        self.publisher.publish(msg)
        #rospy.loginfo("Avg distance: {d}".format(d=msg.data))

    def read_sensor(self, event):
        # read sensor
        distance = self.sensor.read_distance()

        # if we've recorded less than N values
        if len(self.distances) < N:
            self.distances.append(distance)
                
            # if we have N values, publish them
            if len(self.distances) == N:
                self.publish()
            return
            
        # otherwise cycle through list and publish
        self.distances[self.cycle_idx] = distance
        self.cycle_idx += 1
        self.cycle_idx %= N

        # publish
        self.publish()

if __name__ == "__main__":
    try:
        tof_node = TofNode(node_name = "tof_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
