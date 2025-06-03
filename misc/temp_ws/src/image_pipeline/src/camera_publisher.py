#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from image_pipeline.msg import ProcessedImages
import cv2

def main():
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_processed', ProcessedImages, queue_size=1)
    # bridge = CvBridge()

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Could not open camera.")
        return

    rate = rospy.Rate(25)  # 10 Hz
    msg = CompressedImage()
    msg.format = "jpeg"

    processed_msg= ProcessedImages()

    while not rospy.is_shutdown():

        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image")
            continue
        try:
            # Convert the OpenCV image to a ROS CompressedImage message
            success, encoded_image = cv2.imencode(".jpg", frame)
            if success:
                msg.header.stamp = rospy.Time.now()
                msg.data = encoded_image.tobytes()

                processed_msg.raw_image = msg
                processed_msg.undistorted_image = msg
                # Publish the image
                pub.publish(processed_msg)

        except CvBridgeError as e:
            rospy.logerr("Error converting image: {}".format(e))

        # ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # pub.publish(ros_image)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
