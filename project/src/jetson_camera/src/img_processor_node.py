#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

from jetson_camera.msg import ProcessedImages


chessboard_size = (7, 5)

class ImgProcessorNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing image processor node...")
        self.bridge = CvBridge()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.image_cb,
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

        self.print_counter = 0

    def image_cb(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("ImgProcessing node captured first image from publisher.")
        try:
            # Decode image without CvBridge
            raw_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            
            # calibrate the camera
            if not self.calibrated:
                """
                prompt = input("Auto-calibrate camera? (y/N) ")

                # automatic calibration
                if prompt.lower() == "y" or prompt.lower == "yes":
                    self.auto_calibrate_camera()
                    if self.calibrated:
                        return
                """
                
                # manual calibration
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
                    
                    """
                    # save calibration
                    prompt = input("Save calibration settings? (y/N) ")
                    if prompt.lower() == "y" or prompt.lower() == "yes":
                        try:
                            numpy.savetxt("calibration_matrix.csv", self.calibration_matrix, delimiter=",")
                            with open("distortion_coeff.txt", "w") as f:
                                f.write(self.distortion_coeff)
                        except Exception:
                            rospy.logerr("Could not save calibration parameters")
                    """

                return

            # once calibrated, we can begin undistorting and forwarding images
            undistorted_image = self.undistort(raw_image)
            undistorted_image = cv2.resize(undistorted_image, (600, 480))

            # publish images
            msg = ProcessedImages()
            compressed_image_raw = CompressedImage()
            compressed_image_undistorted = CompressedImage()

            compressed_image_raw.format = "jpeg"
            compressed_image_undistorted.format = "jpeg"

            # Convert the OpenCV images to  ROS CompressedImage
            success_raw, encoded_image_raw = cv2.imencode(".jpg", raw_image)
            success_undistorted, encoded_image_undistorted = cv2.imencode(".jpg", undistorted_image)

            if success_raw and success_undistorted:
                compressed_image_raw.data = encoded_image_raw.tobytes()
                compressed_image_undistorted.data = encoded_image_undistorted.tobytes()
                msg.raw_image = compressed_image_raw
                msg.undistorted_image = compressed_image_undistorted

                # Publish the image
                self.pub_image.publish(msg)

                self.print_counter += 1

                if self.print_counter % 100 == 0:
                    rospy.loginfo("Sent 100 processed images")


        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))

    def auto_calibrate_camera(self):
        try:
            self.camera_matrix = np.loadtxt(open("calibration_matrix.csv", "rb"), delimiter=",")
            with open("distortion_coeff.txt") as f:
                self.distortion_coeff = float(f.readline())

            self.calibrated = True
        except Exception:
            rospy.logerr("Could not open calibration files, resorting to manual calibration")

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
        # cv2.imshow('calibration', gray_img)
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
