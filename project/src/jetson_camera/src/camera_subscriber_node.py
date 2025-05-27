#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import ProcessedImages

class CameraSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()
        
        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.image_cb,
            buff_size=2**25,
            queue_size=1
        )

        # Save as video
        # self.video_writer = None
        # self.video_filename = "/home/jetson/camera_output.avi"
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video = cv2.VideoWriter(
            "/home/jetbot/EVC/workshops/workshop2_1/video_undistored.avi",
            self.fourcc,
            15.0,
            (640, 480)
        )
        self.video_raw = cv2.VideoWriter(
            "/home/jetbot/EVC/workshops/workshop2_1/video_raw.avi",
            self.fourcc,
            15.0,
            (1280, 480)
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Camera subscriber node initialized!")

    def image_cb(self, data):
        if not self.initialized:
            return
        
        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Camera subscriber captured first image from publisher.")
        try:
            # Decode image without CvBridge
            cv_image_raw = cv2.imdecode(np.frombuffer(data.raw_image.data, np.uint8), cv2.IMREAD_COLOR)
            cv_image_undistorted = cv2.imdecode(np.frombuffer(data.undistorted_image.data, np.uint8), cv2.IMREAD_COLOR)
            #rospy.loginfo("PubSub delay: {}".format((rospy.Time.now() - data.header.stamp).to_sec()))

            # Save the video
            if cv_image_undistorted is not None:
                rospy.loginfo("receiving undistorted, writing tovideo")
                resized_good = cv2.resize(cv_image_undistorted, (640, 480))
                resized_raw = cv2.resize(cv_image_raw, (640, 480))

                combi = cv2.hconcat([resized_good, resized_raw])
                
                #self.video.write(combi)
                self.video_raw.write(combi)

            # Ensure the window updates instantly
            cv2.imshow("Raw Camera View", cv_image_raw)
            cv2.imshow("Undistorted Camera View", cv_image_undistorted)


            cv2.waitKey(1)  # Keep at 1 to prevent blocking
        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def cleanup(self):
        cv2.destroyAllWindows()
        self.video_raw.release()
        self.video_undistorted.release()

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('camera_subscriber_node', anonymous=True)
    camera_node = CameraSubscriberNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image viewer node.")
    finally:
        camera_node.cleanup()
