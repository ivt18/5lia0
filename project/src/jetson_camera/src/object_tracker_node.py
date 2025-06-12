#!/usr/bin/env python3
import cv2
import os
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from jetson_camera.msg import ProcessedImages, TrackingInfo
from sensor_msgs.msg import CompressedImage
import torch

from ultralytics import YOLO

# model = YOLO('home/ubuntu/.ros/detect/train/weights/best.pt')
# model.conf = 0.4  # confidence threshold

output_img_dir = 'images/val'
output_lbl_dir = 'labels/val'
os.makedirs(output_img_dir, exist_ok=True)
os.makedirs(output_lbl_dir, exist_ok=True)

class ObjectTrackerNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing object tracker node...")
        self.bridge = CvBridge()
        self.frame_id = self.load_frame_id()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.callback,
            buff_size=2**24,
            queue_size=1,
        )

        # Construct publisher
        self.pub_image = rospy.Publisher(
            "camera/image_processed", ProcessedImages, queue_size=1
        )

        self.pub_position = rospy.Publisher(
            "motor_driver/object_tracking", TrackingInfo, queue_size=10
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("Object tracker initialized")

        self.tracker_is_init = False
        self.tracking = False
        self.tracker = None
        self.template = None
        self.target_class = "rc_car"
        self.w = 0
        self.h = 0
        self.data = None
        self.spin()

    def callback(self, data):
        self.data = data

    def spin(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            if self.data is not None:
                self.process_image(self.data)
            rate.sleep()

        # Cleanup
        cv2.destroyAllWindows()

    def process_image(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("Object tracker node captured first image from publisher.")
            rospy.loginfo(cv2.__version__)
        try:
            # Decode image without CvBridge
            undistorted_image = cv2.imdecode(
                np.frombuffer(data.raw_image.data, np.uint8), cv2.IMREAD_COLOR
            )
            
            tracked_image = self.track_image(undistorted_image)

            # publish images
            msg = ProcessedImages()
            compressed_image_raw = CompressedImage()
            compressed_image_undistorted = CompressedImage()

            compressed_image_raw.format = "jpeg"
            compressed_image_undistorted.format = "jpeg"

            # Convert the OpenCV images to  ROS CompressedImage
            success_raw, encoded_image_raw = cv2.imencode(".jpg", undistorted_image)
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
            self.track_object(640, x, y, w, h, True)
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
        cv2.imshow("tracked object", frame)
        cv2.waitKey(1) 
        return frame

    def track_object(self, img_w, x, y, w, h, found):
        kp = -0.00327
        center_x = img_w / 2
        object_x = x + w/2
        error_x = object_x - center_x
       

        angle = kp * error_x
        rospy.loginfo("angle: %s", angle)

        msg = TrackingInfo()
        msg.found = found
        msg.angle = angle
        self.pub_position.publish(msg)

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
                self.track_object(640, x, y, w, h, True)
            else:
                rospy.loginfo("⚠️ Tracker lost target, falling back to detection.")
                self.tracking = False  # tracker failed
                self.track_object(640, 0, 0, 0, 0, False)

        # --- If not tracking or lost, run YOLO detection
        if not self.tracking:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = model(frame_rgb)
            detections = results.pred[0]  # detections for this frame
            self.track_object(640, 0, 0, 0, 0, False)

            for *xyxy, conf, cls in detections:
                class_name = model.names[int(cls)]
                if class_name == target_class:
                    x1, y1, x2, y2 = [int(v) for v in xyxy]
                    bbox = (x1, y1, x2 - x1, y2 - y1)

                    # (Re)initialize tracker
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(frame, bbox)
                    self.tracking = True
                    
                    self.track_object(640, 0, 0, 0, 0, True)

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

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("processing_node", anonymous=True)
    try:
      object_tracker_node = ObjectTrackerNode()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    finally:
        cv2.destroyAllWindows()
