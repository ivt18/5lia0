#!/usr/bin/env python2

import numpy as np
import rospy
from std_msgs.msg import Float64

from jetson_camera.msg import TrackingInfo

class FusionNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        rospy.loginfo("Initializing fusion node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscriber for QR tracking info
        self.sub_qr_tracking = rospy.Subscriber(
            "/motor_driver/qr_tracking",
            TrackingInfo,
            self.qr_tracking_cb,
            queue_size=1
        )

        self.qr_found = False
        self.qr_angle = 0.0
        self.last_time_qr_found = rospy.Time.now()

        # Construct subscriber for object tracking info
        self.sub_object_tracking = rospy.Subscriber(
            "motor_driver/object_tracking",
            TrackingInfo,
            self.object_tracking_cb,
            queue_size=1
        )

        self.object_found = False
        self.object_angle = 0.0
        self.last_time_object_found = rospy.Time.now()

        # Construct publisher
        self.commands_publisher = rospy.Publisher(
            "/camera/fusion",
            Float64,
            queue_size=1
        )

        self.initialized = True
        rospy.loginfo("Fusion node initialized!")

    def qr_tracking_cb(self, msg):
        if not self.initialized:
            return

        # Only continue if QR code tracking is successful
        if not msg.found:
            self.qr_found = False
            return
        
        self.qr_found = True
        self.qr_angle = msg.angle
        self.last_time_qr_found = rospy.Time.now()
        
        self.send_command("qr")

    def object_tracking_cb(self, msg):
        if not self.initialized:
            return

        # Only continue if object tracking is successful
        if not msg.found:
            self.object_found = False
            return
        
        self.object_found = True
        self.object_angle = msg.angle
        self.last_time_object_found = rospy.Time.now()
        
        self.send_command("object")

    def send_command(self, tracking_type):
        # Create a MovementRequest message
        msg = Float64()

        # QR code tracking
        if tracking_type == "qr" and not self.object_found:
            msg.data = self.qr_angle

        # Object tracking
        elif tracking_type == "object" and not self.qr_found:
            msg.data = self.object_angle
        
        # Determine best tracking method
        else:
            if self.qr_found and self.object_found:
                time_since_qr = rospy.Time.now() - self.last_time_qr_found
                time_since_object = rospy.Time.now() - self.last_time_object_found
                total_time = time_since_qr + time_since_object

                new_qr_angle = self.qr_angle
                new_object_angle = self.object_angle

                new_qr_angle = time_since_object / total_time * new_qr_angle
                new_object_angle = time_since_qr / total_time * new_object_angle

                msg.data = new_qr_angle + new_object_angle
            else:
                rospy.logwarn("No valid tracking information available.")
                return
            
        rospy.loginfo("Sending fused angle: {}".format(msg.data))
        
        self.commands_publisher.publish(msg)


if __name__ == "__main__":
    try:
        fusion_node = FusionNode(node_name = "fusion_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
