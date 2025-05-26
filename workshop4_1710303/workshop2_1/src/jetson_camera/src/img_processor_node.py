#!/usr/bin/env python2

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from jetson_camera.msg import ProcessedImages
from sensor_msgs.msg import CompressedImage

chessboard_size = (7, 5)

class ImgProcessorNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.process_image,
            buff_size = 2**24,
            queue_size = 1
        )

        # Construct publisher
        self.pub_image = rospy.Publisher(
            'camera/image_processed',
            ProcessedImages,
            queue_size = 1
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("ImgProcessing node initialized")

        # checkerboard
        self.currcalframes = 0	# number of frames already processed to calibrate the camera
        self.N_CAL_FRAMES = 25	# number of frames we want to use to calibrate the camera
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ...., (7,5,0)
        self.objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        # calibration
        self.calibrated = False
        self.camera_matrix = None
        self.distortion_coeff = None
        self.gray_img = None

        self.tracker_is_init = False
        self.tracker = None
        self.template = None
        self.w = 0
        self.h = 0

    def process_image(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("ImgProcessing node captured first image from publisher.")
            rospy.loginfo(cv2.__version__)
        try:
            # Decode image without CvBridge
            raw_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            # rospy.loginfo("Publisher - ImgProcessor delay: {}".format((rospy.Time.now() - data.header.stamp).to_sec()))

            if not self.calibrated:
                # display raw image
                # cv2.imshow("Raw Camera", raw_image)
                # cv2.waitKey(1)

                if self.currcalframes < self.N_CAL_FRAMES:
                    # record N_CAL_FRAMES of the checkerboard to calibrate
                    rospy.loginfo("Recording checkerboard frame {} / {}".format(self.currcalframes + 1, self.N_CAL_FRAMES))
                    self.gray_img = self.find_chessboard(raw_image)
                    rospy.Rate(2).sleep()
                else:
                    rospy.loginfo("Calibrating...")
                    self.calibrate(self.gray_img)
                    self.calibrated = True
		    cv2.destroyAllWindows()
                    rospy.loginfo("Calibration complete")
                return
            # once calibrated, we can begin undistorting and forwarding images
            undistorted_image = self.undistort(raw_image)
            
            if self.tracker_is_init:
                tracked_image = self.track_image(undistorted_image)
            else:
                self.tracker_init(cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR))
                tracked_image = undistorted_image

            # publish images
            msg = ProcessedImages()
            compressed_image_raw = CompressedImage()
            compressed_image_undistorted = CompressedImage()

            compressed_image_raw.format = "jpeg"
            compressed_image_undistorted.format = "jpeg"

            # Convert the OpenCV images to  ROS CompressedImage
            success_raw, encoded_image_raw = cv2.imencode(".jpg", raw_image)
            success_undistorted, encoded_image_undistorted = cv2.imencode(".jpg", tracked_image)

            if success_raw and success_undistorted:
                compressed_image_raw.data = encoded_image_raw.tobytes()
                compressed_image_undistorted.data = encoded_image_undistorted.tobytes()
                msg.raw_image = compressed_image_raw
                msg.undistorted_image = compressed_image_undistorted

                # Publish the image
                self.pub_image.publish(msg)
                rospy.loginfo("Sent processed image")


        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def tracker_init(self, first_frame):
        # Createtracker
        # === Convert to HSV and create mask for red ===
        hsv = cv2.cvtColor(first_frame, cv2.COLOR_BGR2HSV)

        # Red color range (can tweak for lighting)
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([5, 255, 255])
        lower_red2 = np.array([170, 150, 150])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        # Find contours and pick largest red object
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("No red object found.")
            return;

        # Assume the largest red object is the target
        contour = max(contours, key=cv2.contourArea)
        x, y, self.w, self.h = cv2.boundingRect(contour)
        self.template = first_frame[y:y+self.h, x:x+self.w]
        self.tracker_is_init = True
        rospy.loginfo("Tracker initialized.")
        return

    def track_image(self, frame):
        if frame is not None:
            result = cv2.matchTemplate(frame, self.template, cv2.TM_CCOEFF_NORMED)
            _, _, _, max_loc = cv2.minMaxLoc(result)
            top_left = max_loc
            bottom_right = (top_left[0] + self.w, top_left[1] + self.h)
            cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
 
        return frame

    def find_chessboard(self, raw_image):
        gray_img = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray_img, chessboard_size, None)
        # rospy.loginfo("Found chessboard? {}".format(ret))

        # If found, add object points, image points (after refining them)
        if ret == True:
            self.objpoints.append(self.objp)

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray_img, corners, (11, 11), (-1, -1), criteria)
            self.imgpoints.append(corners2)

            # update
            self.currcalframes += 1

            # Draw and display the corners
            cv2.drawChessboardCorners(gray_img, chessboard_size, corners2, ret)
            cv2.imshow('calibration', gray_img)
            cv2.waitKey(1)
        return gray_img

    def calibrate(self, gray_image):
        ret, self.camera_matrix, self.distortion_coeff, rotation_vecs, translation_vecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray_image.shape[::-1], None, None)

    def undistort(self, image):
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion_coeff, (w, h), 1, (w, h))

        # undistort image
        dst = cv2.undistort(image, self.camera_matrix, self.distortion_coeff, None, new_camera_matrix)

        # crop image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('processing_node', anonymous=True)
    img_processor_node = ImgProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    finally:
        cv2.destroyAllWindows()
