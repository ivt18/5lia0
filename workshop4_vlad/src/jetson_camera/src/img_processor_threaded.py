#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import threading
import queue
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from jetson_camera.msg import ProcessedImages

chessboard_size = (7, 5)

class ImgProcessorNode:
    def __init__(self):
        rospy.loginfo("Initializing ImgProcessorNode...")

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=1)
        self.initialized = True

        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.image_cb,
            queue_size=1,
            buff_size=2**24
        )

        self.pub_image = rospy.Publisher(
            "camera/image_processed_vlad",
            ProcessedImages,
            queue_size=1
        )

        # Calibration setup
        self.currcalframes = 0
        self.N_CAL_FRAMES = 25
        self.objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        self.calibrated = False
        self.camera_matrix = None
        self.distortion_coeff = None
        self.gray_img = None

        # Processing thread
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        rospy.loginfo("ImgProcessorNode initialized.")

    def image_cb(self, data):
        if not self.initialized:
            return

        rospy.loginfo("Received image data")
        try:
            raw_image = cv2.imdecode(np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR)
            self.image_queue.put_nowait(raw_image)
        except queue.Full:
            pass  # Drop old image if processing can't keep up

    def process_images(self):
        while not rospy.is_shutdown():
            try:
                image = self.image_queue.get(timeout=1)

                if not self.calibrated:
                    if self.currcalframes < self.N_CAL_FRAMES:
                        self.gray_img = self.find_chessboard(image)
                        continue
                    else:
                        rospy.loginfo("Calibrating...")
                        self.calibrate(self.gray_img)
                        self.calibrated = True
                        cv2.destroyAllWindows()
                        rospy.loginfo("Calibration complete")
                        continue

                undistorted_image = self.undistort(image)

                msg = ProcessedImages()
                compressed_image_raw = CompressedImage()
                compressed_image_undistorted = CompressedImage()

                compressed_image_raw.format = "jpeg"
                compressed_image_undistorted.format = "jpeg"

                success_raw, encoded_raw = cv2.imencode(".jpg", image)
                success_undist, encoded_undist = cv2.imencode(".jpg", undistorted_image)

                if success_raw and success_undist:
                    compressed_image_raw.data = encoded_raw.tobytes()
                    compressed_image_undistorted.data = encoded_undist.tobytes()
                    msg.raw_image = compressed_image_raw
                    msg.undistorted_image = compressed_image_undistorted

                    self.pub_image.publish(msg)

            except queue.Empty:
                continue

    def find_chessboard(self, image):
        gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_img, chessboard_size, None)
        if ret:
            self.objpoints.append(self.objp)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray_img, corners, (11, 11), (-1, -1), criteria)
            self.imgpoints.append(corners2)
            self.currcalframes += 1
            cv2.drawChessboardCorners(gray_img, chessboard_size, corners2, ret)
            cv2.imshow("calibration", gray_img)
            cv2.waitKey(1)
        return gray_img

    def calibrate(self, gray_image):
        _, self.camera_matrix, self.distortion_coeff, _, _ = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, gray_image.shape[::-1], None, None)

    def undistort(self, image):
        h, w = image.shape[:2]
        new_cam_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.distortion_coeff, (w, h), 1, (w, h))
        dst = cv2.undistort(image, self.camera_matrix, self.distortion_coeff, None, new_cam_matrix)
        x, y, w, h = roi
        return dst[y:y+h, x:x+w]

if __name__ == "__main__":
    rospy.init_node("img_processor_node_vlad", anonymous=True)
    node = ImgProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ImgProcessorNode")
    finally:
        cv2.destroyAllWindows()
