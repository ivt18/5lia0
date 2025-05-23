#!/usr/bin/env python2

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from jetson_camera.msg import ProcessedImages
from jetson_camera.msg import MovementRequest


chessboard_size = (7, 5)

class QRCodeNode:
    def __init__(self):
        self.initialized = False
        rospy.loginfo("Initializing qr code node...")
        self.bridge = CvBridge()

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.image_cb,
            buff_size = 2**25,
            queue_size = 1
        )

        # Construct publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/commands",
            MovementRequest,
            queue_size=10,
        )

        # Create QR Code Reader
        self.qr_code_detector = cv2.QRCodeDetector()

        # Save as video
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.video = cv2.VideoWriter(
            "/home/jetbot/EVC/5lia0/workshop3_1561111/video_qr_code.avi",
            self.fourcc,
            15.0,
            (1280, 480)
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("QRCode node initialized")

    def send_command(self, found, data=""):
        if not self.initialized:
            return

        # Process the QR code data if found
        if found:

            rospy.loginfo("QR code: %s", data)

            msg = MovementRequest()

            if data == "forward":
                msg.request_type = 1
                msg.value = 0.1
            
            elif data == "reverse":
                msg.request_type = 1
                msg.value = -0.1

            elif data == "rotate left":
                msg.request_type = 2
                msg.value = -0.1

            elif data == "rotate right":
                msg.request_type = 2
                msg.value = 0.1

            else:
                rospy.logwarn("Unknown command: %s", data)
                return
            
            self.publisher.publish(msg)

        rospy.loginfo("Sent command to controller: found = {}, data = {}".format(found, data))

    def image_cb(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("QRCode node captured first image from publisher.")
        try:
            # Decode image without CvBridge
            cv_image_raw = cv2.imdecode(np.frombuffer(data.raw_image.data, np.uint8), cv2.IMREAD_COLOR)
            undistorted_image = cv2.imdecode(np.frombuffer(data.undistorted_image.data, np.uint8), cv2.IMREAD_COLOR)

            if undistorted_image is not None and cv_image_raw is not None:

                # Try to detect QR codes and decode if found
                retval, points, straight_qrcode = self.qr_code_detector.detectAndDecode(undistorted_image)
                
                # Check if a QR code was detected
                if retval:
                    # Display the decoded data from the QR code
                    cv2.putText(undistorted_image, retval, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.polylines(undistorted_image, [points.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)

                    # Drive forward if QR code detected
                    self.send_command(True, data=retval)
                else:
                    # Display if no QR code was detected
                    cv2.putText(undistorted_image, "No QR code detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # Send command that no QR code was found
                    self.send_command(False)

                # Save the video
                rospy.loginfo("receiving undistorted, writing tovideo")
                resized_good = cv2.resize(undistorted_image, (640, 480))
                resized_raw = cv2.resize(cv_image_raw, (640, 480))

                combi = cv2.hconcat([resized_good, resized_raw])
                
                self.video.write(combi)

                cv2.waitKey(1)  # Keep at 1 to prevent blocking


        except CvBridgeError as err:
            rospy.logerr("Error converting image: {}".format(err))


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('qr_code_node', anonymous=True)
    qr_code_node = QRCodeNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image processor node.")
    finally:
        cv2.destroyAllWindows()
