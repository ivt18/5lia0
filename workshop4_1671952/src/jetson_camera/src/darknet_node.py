#!/usr/bin/env python2

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import ProcessedImages
from controller_package.msg import MovementRequest

from darknet import darknet

class DarknetNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing darknet node...")
        self.bridge = CvBridge()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.image_cb,
            buff_size = 2**25,
            queue_size = 1
        )

        # Construct publisher
        self.pub_commands = rospy.Publisher(
            "motor_driver/commands",
            MovementRequest,
            queue_size=10
        )

        self.first_image_received = False

        # YOLO model
        self.cfg_path = "/home/jetbot/EVC/workshops/workshop2_1671952/src/jetson_camera/src/darknet/cfg/yolov4-tiny.cfg"
        self.data_path = "/home/jetbot/EVC/workshops/workshop2_1671952/src/jetson_camera/src/darknet/cfg/coco-ros.data"
        self.weights_path = "/home/jetbot/EVC/workshops/workshop2_1671952/src/jetson_camera/src/darknet/yolov4-tiny.weights"
        self.net, self.class_names, self.class_colors = darknet.load_network(
            self.cfg_path, self.data_path, self.weights_path, batch_size=1)
        self.net_width = darknet.network_width(self.net)
        self.net_height = darknet.network_height(self.net)

        self.initialized = True
        rospy.loginfo("Darknet node initialized")

    def image_cb(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Darknet node captured first image from publisher.")
        try:
            # Get undistorted image
            cv_image_bgr = cv2.imdecode(np.frombuffer(data.undistorted_image.data, np.uint8), cv2.IMREAD_COLOR)
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)

            # Resize image to network input size
            cv_image_resized = cv2.resize(cv_image_rgb, (self.net_width, self.net_height))

            # Convert image to Darknet format and run it through network
            darknet_img = darknet.make_image(self.net_width, self.net_height, 3)
            darknet.copy_image_from_bytes(darknet_img, cv_image_resized.tobytes())
            detections = darknet.detect_image(self.net, self.class_names, darknet_img)
            darknet.free_image(darknet_img)

            # Sort detections
            detections_sorted = sorted(detections, key=lambda det: float(det[1]), reverse=True)

            # Draw detections
            for label, confidence, bbox in detections_sorted:
                x, y, w, h = map(int, bbox)
                left = int(x - w / 2)
                top = int(y - h / 2)
                right = int(x + w / 2)
                bottom = int(y + h / 2)

                cv2.rectangle(
                    cv_image_resized,
                    (left, top),
                    (right, bottom),
                    self.class_colors[label],
                    2)
                cv2.putText(cv_image_resized, "{} [{}]".format(label, confidence),
                    (left, top - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    self.class_colors[label],
                    2)

            cv_image_bgr_detections = cv2.cvtColor(cv_image_resized, cv2.COLOR_RGB2BGR)
            cv2.imshow("Detections", cv_image_bgr_detections)
            cv2.imshow("Undistorted", cv_image_bgr)
            cv2.waitKey(1)

            # Rotate robot towards the detection with highest confidence
            if detections_sorted:
                top_label, top_confidence, top_bbox = detections_sorted[0]
                x, y, w, h = map(int, bbox)
                delta = float(x - float(self.net_width / 2))
                delta_norm = float(2 * delta / float(self.net_width))
                rospy.loginfo("delta_norm: {}".format(delta_norm))
                msg = MovementRequest()
                msg.request_type = 2
                msg.value = 0.1 * delta_norm
                self.pub_commands.publish(msg)

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('darknet_node', anonymous=True)
    img_processor_node = DarknetNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    finally:
        cv2.destroyAllWindows()
