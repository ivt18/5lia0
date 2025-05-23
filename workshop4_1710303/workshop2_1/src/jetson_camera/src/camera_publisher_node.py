#!/usr/bin/env python2

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class CameraPublisherNode:
    def __init__(self, node_name):
        self.initialized = False
        rospy.loginfo("Initializing camera publisher node...")
        # Initialize the ROS node
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # Publisher for the image topic
        self.image_pub = rospy.Publisher(
            '/camera/image_raw', 
            CompressedImage,
            queue_size=1)

        # Create a CvBridge object for converting images
        self.bridge = CvBridge()

        # Camera Dimensions And Properties Parameters
        config = self.load_yaml()

        self.sensor_id = config["sensor_id"]  # 0: CSI, 1: USB
        self.width = config["width"]
        self.height = config["height"]
        self.fps = config["fps"]
        
        # GStreamer pipeline for the Jetson CSI camera
        self.pipeline = self.gstreamer_pipeline()
        if self.pipeline is None:
            rospy.logerr("Pipeline could not be initialized!")

        # OpenCV video capture with the GStreamer pipeline
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            rospy.logerr("Unable to open camera")

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera publisher node initialized!")

    def gstreamer_pipeline(self):
        if self.sensor_id == 0:
            return ('nvarguscamerasrc sensor-id=0 ! '
                    'video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, '
                    'format=(string)NV12, framerate=(fraction)%d/1 ! '
                    'queue max-size-buffers=1 leaky=downstream ! '
                    'nvvidconv ! video/x-raw, format=(string)I420 ! '
                    'videoconvert ! video/x-raw, format=(string)BGR ! '
                    'queue max-size-buffers=1 leaky=downstream ! '
                    'appsink drop=true sync=false' % (self.width, self.height, self.fps))

        elif self.sensor_id == 1:
            return ('v4l2src device=/dev/video1 ! '
                    'video/x-raw, width=(int)%d, height=(int)%d, '
                    'format=(string)YUY2, framerate=(fraction)%d/1 ! '
                    'videoconvert ! video/x-raw, format=(string)BGR ! '
                    'appsink' % (self.width, self.height, self.fps))
        else:
            return None

    def start_publishing(self):
        rate = rospy.Rate(self.fps)
        msg = CompressedImage()
        msg.format = "jpeg"

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture image")
                continue
            if self.first_image_received == False:
                self.first_image_received = True
                rospy.loginfo("Camera publisher captured first image from physical device.")
            try:
                # Convert the OpenCV image to a ROS CompressedImage message
                success, encoded_image = cv2.imencode(".jpg", frame)
                if success:
                    msg.header.stamp = rospy.Time.now()
                    msg.data = encoded_image.tobytes()
                    # Publish the image
                    self.image_pub.publish(msg)

            except CvBridgeError as e:
                rospy.logerr("Error converting image: {}".format(e))

            # Sleep to maintain the publishing rate
            rate.sleep()

    def cleanup(self):
        self.cap.release()

    def load_yaml(self):
        config = {}
        config["sensor_id"] = rospy.get_param("~sensor_id")
        config["width"] = rospy.get_param("~width")
        config["height"] = rospy.get_param("~height")
        config["fps"] = rospy.get_param("~fps")

        # Log the loaded configuration
        rospy.loginfo("Loaded config: %s", config)

        return config

if __name__ == "__main__":
    node_name = "camera_publisher_node"
    camera_pub = None
    try:
        camera_pub = CameraPublisherNode(node_name)
        camera_pub.start_publishing()
    except rospy.ROSInterruptException:
        pass
    finally:
        if camera_pub is not None:
            camera_pub.cleanup()
