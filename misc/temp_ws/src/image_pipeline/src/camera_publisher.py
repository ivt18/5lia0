#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from image_pipeline.msg import ProcessedImages
import cv2
import os

def main():
    rospy.init_node('camera_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_processed', ProcessedImages, queue_size=10)
    # bridge = CvBridge()

    video_path = os.path.expanduser("~/EVC/vision_video.mp4")

    if not os.path.exists(video_path):
        rospy.logerr("Video file does not exist.")
        exit(1)

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("Could not open camera.")
        return

    rate = rospy.Rate(20)  # 10 Hz
    msg = CompressedImage()
    msg.format = "jpeg"

    processed_msg= ProcessedImages()

    while not rospy.is_shutdown():

        ret, frame = cap.read()
        if not ret:
            # rospy.logwarn("Failed to capture image")
            rospy.loginfo("restarting video loop woop")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        try:
            # rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            # Convert the OpenCV image to a ROS CompressedImage message
            success, encoded_image = cv2.imencode(".jpg", frame)
            if success:
                # cv2.imshow("Camera subscriber", frame)
                # cv2.waitKey(1)

                msg.header.stamp = rospy.Time.now()
                msg.data = encoded_image.tobytes()

                processed_msg.raw_image = msg
                processed_msg.undistorted_image = msg
                # Publish the image
                pub.publish(processed_msg)
                rate.sleep()



        except CvBridgeError as e:
            rospy.logerr("Error converting image: {}".format(e))

        # ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # pub.publish(ros_image)

        # rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
