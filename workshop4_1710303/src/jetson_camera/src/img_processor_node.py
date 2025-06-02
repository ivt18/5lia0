#!/usr/bin/env python3
import cv2
import os
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from jetson_camera_tracker.msg import ProcessedImages, ObjectPosition
from sensor_msgs.msg import CompressedImage
import torch

from ultralytics import YOLO

# model = YOLO('home/ubuntu/.ros/detect/train/weights/best.pt')
# model.conf = 0.4  # confidence threshold

chessboard_size = (7, 5)
output_img_dir = 'images/val'
output_lbl_dir = 'labels/val'
os.makedirs(output_img_dir, exist_ok=True)
os.makedirs(output_lbl_dir, exist_ok=True)


class ImgProcessorNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing camera subscriber node...")
        self.bridge = CvBridge()
        self.frame_id = self.load_frame_id()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_raw",
            CompressedImage,
            self.process_image,
            buff_size=2**24,
            queue_size=1,
        )

        # Construct publisher
        self.pub_image = rospy.Publisher(
            "camera/image_processed", ProcessedImages, queue_size=1
        )

        self.pub_position = rospy.Publisher(
            "camera/object_position", ObjectPosition, queue_size=10
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("ImgProcessing node initialized")

        # checkerboard
        self.currcalframes = (
            0  # number of frames already processed to calibrate the camera
        )
        self.N_CAL_FRAMES = (
            25  # number of frames we want to use to calibrate the camera
        )
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ...., (7,5,0)
        self.objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            0 : chessboard_size[0], 0 : chessboard_size[1]
        ].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        # calibration
        self.calibrated = False
        self.camera_matrix = None
        self.distortion_coeff = None
        self.gray_img = None

        self.tracker_is_init = False
        self.tracking = False
        self.tracker = None
        self.template = None
        self.target_class = "rc_car"
        self.w = 0
        self.h = 0
        self.object_position = ObjectPosition()

    def process_image(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("ImgProcessing node captured first image from publisher.")
            rospy.loginfo(cv2.__version__)
        try:
            # Decode image without CvBridge
            raw_image = cv2.imdecode(
                np.frombuffer(data.data, np.uint8), cv2.IMREAD_COLOR
            )
            print("Current working directory:", os.getcwd())
            
            # rospy.loginfo("Publisher - ImgProcessor delay: {}".format((rospy.Time.now() - data.header.stamp).to_sec()))

            if not self.calibrated:
                # display raw image
                # cv2.imshow("Raw Camera", raw_image)
                # cv2.waitKey(1)

                if self.currcalframes < self.N_CAL_FRAMES:
                    # record N_CAL_FRAMES of the checkerboard to calibrate
                    rospy.loginfo(
                        "Recording checkerboard frame {} / {}".format(
                            self.currcalframes + 1, self.N_CAL_FRAMES
                        )
                    )
                    self.gray_img = self.find_chessboard(raw_image)
                    rospy.loginfo(raw_image.shape)
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
            tracked_image = self.track_image(undistorted_image)

            # publish images
            msg = ProcessedImages()
            compressed_image_raw = CompressedImage()
            compressed_image_undistorted = CompressedImage()

            compressed_image_raw.format = "jpeg"
            compressed_image_undistorted.format = "jpeg"

            # Convert the OpenCV images to  ROS CompressedImage
            success_raw, encoded_image_raw = cv2.imencode(".jpg", raw_image)
            success_undistorted, encoded_image_undistorted = cv2.imencode(
                ".jpg", tracked_image
            )

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
        self.tracker = cv2.TrackerCSRT_create()  # or TrackerKCF_create(), etc.

        # Define initial bounding box (e.g., manually or from detection)
        bbox = cv2.selectROI("Frame", first_frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Frame")

        # Initialize the tracker with the first frame and bounding box
        self.tracker.init(first_frame, bbox)
        self.tracker_is_init = True
        rospy.loginfo("Tracker initialized.")
        return

    def track_image(self, frame):
        if not self.tracker_is_init:
            self.tracker_init(frame)
        if frame is None:
            return  # Stop if no frame is received

        # Update tracker
        success, bbox = self.tracker.update(frame)
        
        # Draw bounding box
        if success:
            (x, y, w, h) = [int(v) for v in bbox]
            frame_raw = frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.object_position.x = x
            self.object_position.y = y
            self.object_position.width = w
            self.object_position.height = h
            self.object_position.image_width = frame.shape[1]

            # self.store_image(frame_raw, x, y, w, h)
        else:
            cv2.putText(
                frame,
                "Tracking failure",
                (50, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )

        return frame

    def track_object(self, img_w, x, y, w, h):
        kp = 0.05
        center_x = img_w / 2
        object_x = x + w/2
        error_x = object_x - center_x
        rospy.loginfo("error_x: %s", error_x)
        
        if error_x > 0 or error_x < -20:
            angle = kp * error_x

    def store_image(self, frame_raw, x, y, w, h):
        img_h, img_w = frame_raw.shape[:2]

        # Normalize for YOLO format
        cx = (x + w / 2) / img_w
        cy = (y + h / 2) / img_h
        nw = w / img_w 
        nh = h / img_h

        # Save image and label
        img_name = f"{self.frame_id:05d}.jpg"
        lbl_name = f"{self.frame_id:05d}.txt"

        cv2.imwrite(os.path.join(output_img_dir, img_name), frame_raw)
        with open(os.path.join(output_lbl_dir, lbl_name), 'w') as f:
            f.write(f"0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        rospy.loginfo("Wrote image %s", self.frame_id)

        self.frame_id += 1
        self.save_frame_id(self.frame_id)

    def detector_track(self, frame):
        if self.tracking:
            success, bbox = self.tracker.update(frame)
            if success:
                x, y, w, h = [int(v) for v in bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            else:
                rospy.loginfo("⚠️ Tracker lost target, falling back to detection.")
                self.tracking = False  # tracker failed

        # --- If not tracking or lost, run YOLO detection
        if not self.tracking:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = model(frame_rgb)
            detections = results.pred[0]  # detections for this frame

            for *xyxy, conf, cls in detections:
                class_name = model.names[int(cls)]
                if class_name == target_class:
                    x1, y1, x2, y2 = [int(v) for v in xyxy]
                    bbox = (x1, y1, x2 - x1, y2 - y1)

                    # (Re)initialize tracker
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(frame, bbox)
                    self.tracking = True

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    break
        return frame


    def load_frame_id(self, path='frame_index.txt'):
        try:
            with open(path, 'r') as f:
                return int(f.read())
        except:
            return 0  # Start from 0 if file doesn't exist

    def save_frame_id(self, frame_id, path='frame_index.txt'):
        with open(path, 'w') as f:
            f.write(str(frame_id))

    def find_chessboard(self, raw_image):
        gray_img = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray_img, chessboard_size, None)
        rospy.loginfo("Found chessboard? {}".format(ret))
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
            cv2.imshow("calibration", gray_img)
            cv2.waitKey(1)
        return gray_img

    def calibrate(self, gray_image):
        (
            ret,
            self.camera_matrix,
            self.distortion_coeff,
            rotation_vecs,
            translation_vecs,
        ) = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, gray_image.shape[::-1], None, None
        )

    def undistort(self, image):
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.distortion_coeff, (w, h), 1, (w, h)
        )

        # undistort image
        dst = cv2.undistort(
            image, self.camera_matrix, self.distortion_coeff, None, new_camera_matrix
        )

        # crop image
        x, y, w, h = roi
        dst = dst[y : y + h, x : x + w]

        return dst


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("processing_node", anonymous=True)
    img_processor_node = ImgProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    finally:
        cv2.destroyAllWindows()
