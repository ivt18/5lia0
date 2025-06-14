#!/usr/bin/env python3
import cv2
import os
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from jetson_camera.msg import ProcessedImages, TrackingInfo
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

# model = YOLO('/home/ubuntu/5lia0/yolo/runs/detect/rc_car_model3/weights/best.pt')
model = YOLO("yolov8n.pt")
model.conf = 0.1  # confidence threshold
output_img_dir = 'images/temp'
output_lbl_dir = 'labels/temp'
os.makedirs(output_img_dir, exist_ok=True)
os.makedirs(output_lbl_dir, exist_ok=True)

class ObjectTrackerNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing object tracker node...")
        print(os.getcwd())
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
        self.ctr = 638
        self.mode = "manual"
        self.train = False

    # updates the image upon receiving a new one
    def callback(self, data):
        self.data = data

    # continuously processes incoming images
    def spin(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            if self.data is not None:
                self.process_image(self.data)
            rate.sleep()

        # Cleanup
        cv2.destroyAllWindows()

    # top-level function to process images
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
            # test_image = cv2.imread(f"images/val/00{self.ctr}.jpg", cv2.IMREAD_COLOR) 
            self.ctr += 1

            if self.mode == "auto":
                self.detector_track(undistorted_image)
            else:
                self.track_image(undistorted_image)
        except:
            rospy.loginfo("failed to track image")

    # initialize tracker manually by selecting bounding box
    def tracker_init(self, first_frame):
        # Createtracker
        self.tracker = cv2.TrackerCSRT_create()

        # Define initial bounding box (e.g., manually or from detection)
        bbox = cv2.selectROI("Frame", first_frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Frame")

        # Initialize the tracker with the first frame and bounding box
        self.tracker.init(first_frame, bbox)
        self.tracker_is_init = True
        rospy.loginfo("Tracker initialized.")

    # track object without object detection (might lose track when object leaves frame)
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
            self.send_angle(640, x, y, w, h, True)
            if self.train:
                self.store_image(frame_raw, x, y, w, h)
            cv2.imshow("tracked object", frame)
            cv2.waitKey(1) 
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
            self.tracker_is_init = False
            self.tracker_init(frame);
    
    # track image with NN object detection
    def detector_track(self, frame):
        # if we lost track of the object or every 30 frames, re-detect object
        if self.tracking and not (self.ctr % 30 == 0):
            success, bbox = self.tracker.update(frame)
            if success:
                x, y, w, h = [int(v) for v in bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.send_angle(640, x, y, w, h, True)
            else:
                rospy.loginfo("Tracker lost target, falling back to detection.")
                self.tracking = False
                self.send_angle(640, 320, 0, 0, 0, False)

        else:
            rospy.loginfo("detecting")

            results = model(frame)
            boxes = results[0].boxes
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            if boxes is not None and len(boxes) > 0:
                # Get index of box with highest confidence
                best_idx = boxes.conf.argmax()

                # Extract best box
                best_box = boxes[best_idx]

                x1, y1, x2, y2 = best_box.xyxy[0].tolist()
                bbox = (int(x1), int(y1), int(x2-x1), int(y2-y1))

                # (Re)initialize tracker
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(frame, bbox)
                self.tracking = True
                self.send_angle(640, int(x1), int(y1), int(x2-x1), int(y2-y1), True)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
            else:
                self.send_angle(640, 320, 0, 0, 0, False)
        
        cv2.imshow("tracked object", frame)
        cv2.waitKey(1)

    #  find object angle and send it to fusion node
    def send_angle(self, img_w, x, y, w, h, found):
        kp = -0.0010
        center_x = img_w / 2
        object_x = x + w/2
        error_x = object_x - center_x
       
        angle = (kp * error_x)
        rospy.loginfo("object angle: %s", angle)

        msg = TrackingInfo()
        msg.found = found
        msg.angle = angle
        self.pub_position.publish(msg)

    # stores image with corresponding label to train the NN
    def store_image(self, frame_raw, x, y, w, h):
        img_h, img_w = frame_raw.shape[:2]

        # Normalize for YOLO format
        cx = (x + w / 2) / img_w
        cy = (y + h / 2) / img_h
        nw = w / img_w 
        nh = h / img_h
        self.frame_id = self.load_frame_id()

        # Save image and label
        img_name = f"{self.frame_id:05d}.jpg"
        lbl_name = f"{self.frame_id:05d}.txt"

        cv2.imwrite(os.path.join(output_img_dir, img_name), frame_raw)
        with open(os.path.join(output_lbl_dir, lbl_name), 'w') as f:
            f.write(f"0 {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        rospy.loginfo("Wrote image %s", self.frame_id)

        self.frame_id += 1
        self.save_frame_id(self.frame_id)

    # loads id of next frame to store
    # used if data collection is stopped and needs to be resumed later
    def load_frame_id(self, path='frame_index.txt'):
        try:
            with open(path, 'r') as f:
                return int(f.read())
        except:
            return 0  # Start from 0 if file doesn't exist

    # saves frame id of next frame to store
    # used if data collection is stopped and needs to be resumed later
    def save_frame_id(self, frame_id, path='frame_index.txt'):
        with open(path, 'w') as f:
            f.write(str(frame_id))

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node("processing_node", anonymous=True)
    try:
      object_tracker_node = ObjectTrackerNode()
      object_tracker_node.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    finally:
        cv2.destroyAllWindows()
