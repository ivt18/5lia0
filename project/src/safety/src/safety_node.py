#!/usr/bin/env python2

import time

import rospy

from std_msgs.msg import Bool

from enums import StopGo, StopSign


STOP_SIGN_TIME = 3      # time (sec) to serve a stop sign


class SafetyNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {name} node...".format(name=self.node_name))
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscribers
        self.text_recognition = rospy.Subscriber(
            "/camera/text",
            Bool,
            self.text_cb,
            buff_size=10,
            queue_size=1,
        )

        self.pedestrian_detection = rospy.Subscriber(
            "/camera/pedestrian",
            Bool,
            self.pedestrian_cb,
            buff_size=10,
            queue_size=1,
        )

        # Construct publishers
        self.publisher = rospy.Publisher(
            "/safety/stop_go",
            Bool,
            queue_size=1,
        )

        # current state of all stop/go signals
        self.signals = {"text": StopGo.GO, "pedestrian": StopGo.GO}
        self.stop_sign_state = StopSign.GO
        
        # time when the last stop sign began to be served
        self.stop_sign_start = -STOP_SIGN_TIME

        self.initialized = True
        rospy.loginfo("Node {name} initialized!".format(name=self.node_name))
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish)


    def text_cb(self, data):
        """
        Handling text signal is a bit different. We want to stop for STOP_TIME seconds upon reception of a stop signal.
        All subsequent STOP signals without an intermittent GO signal are ignored, as we've already served this STOP.
        Upon reception of a GO signal, the state is reset.
        """
        if not self.initialized:
            return
        
        signal = data.data

        rospy.loginfo("Received text signal: {signal}".format(signal=StopGo(signal)))

        # if we receive a STOP while in GO, this is a new stop sign -> begin stop sequence
        if signal == StopGo.STOP.value and self.stop_sign_state == StopSign.GO:
            self.stop_sign_start = time.time()
            self.stop_sign_state = StopSign.STOP
        # if we receive a GO while leaving a STOP, then the STOP sign is no longer visible and we can reset
        elif signal == StopGo.GO.value and self.stop_sign_state == StopSign.LEAVING:
            self.stop_sign_state = StopSign.GO
        
        rospy.loginfo("stop sign state: {signal}".format(signal=StopGo(self.stop_sign_state)))


    def pedestrian_cb(self, data):
        if not self.initialized:
            return

        self.signals["pedestrian"] = StopGo(data.data)

    
    def publish(self, event):
        msg = Bool()

        time_served = time.time() - self.stop_sign_start    # time passed since we first saw the current stop sign

        # compute state of text signal
        # if we have not finished serving the last stop sign, send STOP
        if time_served < STOP_SIGN_TIME:
            self.signals["text"] = StopGo.STOP
        else:
            self.signals["text"] = StopGo.GO
            if self.stop_sign_state == StopSign.STOP:
                self.stop_sign_state = StopSign.LEAVING
        """
        - if we have finished serving the last stop sign, and are currently in GO, send GO

        - if we have finished serving the last stop sign, and are currently in STOP, this is the same stop signal
            we just finished serving, so send GO
        """

        # signal sent is the disjunction of all signals (i.e. if at least one signal is STOP then we send STOP, otherwise send GO)
        # msg.data = any(state == StopGo.STOP for state in self.signals.values())   # no clue why this doesnt work :(
        msg.data = StopGo.GO.value
        for state in self.signals.values():
            if state == StopGo.STOP:
                msg.data = StopGo.STOP.value
                break

        # publish signal
        self.publisher.publish(msg)
        rospy.loginfo("Sent signal {signal}".format(signal=StopGo(msg.data)))

if __name__ == "__main__":
    try:
        safety_node = SafetyNode(node_name = "safety_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
