#!/usr/bin/env python2

import numpy as np
import rospy

from jetson_camera.msg import QRTrackingInfo
from motor_driver_package.msg import MovementRequest

class FusionNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name

        rospy.loginfo("Initializing receiver node...")
        rospy.init_node(self.node_name, anonymous=True)

        # Construct subscriber for QR tracking info
        self.sub_qr_tracking_info = rospy.Subscriber(
            "/motor_driver/qr_tracking_info",
            QRTrackingInfo,
            self.qr_tracking_info_cb,
            queue_size=10
        )

        # Construct publisher
        self.commands_publisher = rospy.Publisher(
            "/motor_driver/commands",
            MovementRequest,
            queue_size=10
        )

        self.initialized = True
        rospy.loginfo("Receiver node initialized!")

    def qr_tracking_info_cb(self, msg):
        if not self.initialized:
            return

        # Extract data from the message
        if not msg.found:
            return
        
        qr_offset = msg.offset
        qr_distance = msg.distance
        rospy.loginfo("QR Code found at offset: %d, distance: %f", qr_offset, qr_distance)

        # Create a MovementRequest message
        msg = MovementRequest()

        if qr_offset < -0.5: # Rotate left
            rospy.loginfo("Rotating left with measured offset: %d", qr_offset)
            msg.request_type = 2
            msg.value = -0.1

        elif qr_offset > 0.5: # Rotate right
            rospy.loginfo("Rotating right with measured offset: %d", qr_offset)
            msg.request_type = 2
            msg.value = 0.1

        elif qr_distance < 0.01: # Move forward
            rospy.loginfo("Moving forward with measured distance: %f", qr_distance)
            msg.request_type = 1
            msg.value = 0.1
        
        elif qr_distance > 0.01: # Move backward
            rospy.loginfo("Moving backward with measured distance: %f", qr_distance)
            msg.request_type = 1
            msg.value = -0.1

        else: # Ignore
            return
            
        self.commands_publisher.publish(msg)

if __name__ == "__main__":
    try:
        fusion_node = FusionNode(node_name = "fusion_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
