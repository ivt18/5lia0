#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from jetson_camera.msg import ProcessedImages
import os
import socket
import struct

ocr_server_ip = "192.168.8.169"
class OcrCompressedNode:
    def __init__(self):
        rospy.init_node('ocr_node', anonymous=True)
        rospy.loginfo("OCR Node (CompressedImage) initialized.")
        self.s = socket.socket()
        self.s.connect((ocr_server_ip, 9999)) 
        rospy.loginfo("Connected to socket server.")
        
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.image_cb,
            buff_size=2**25,
            queue_size=1
        )
        # TODO: uncomment this when done
        rospy.spin()

    def send_image_over_socket(self, image_data):
        # rospy.loginfo("Sending image over socket...")
        try:
            length = struct.pack('>I', len(image_data))
            self.s.sendall(length + image_data)
        except Exception as e:
            rospy.logerr("Socket error: {e}".format(e))

    def image_cb(self, msg):
        # rospy.Rate(10).sleep()  
        image_data = msg.undistorted_image.data
        try:
            np_arr = np.frombuffer(msg.raw_image.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # self.extract_text(cv2.imdecode(np.frombuffer(msg.undistorted_image.data, np.uint8), cv2.IMREAD_COLOR))
            self.send_image_over_socket(image_data)
            cv2.imshow("Raw Image", image)
            cv2.waitKey(1)  

            # text = pytesseract.image_to_string(image)

            # rospy.loginfo(f"OCR Text: {text.strip()}")

        except Exception as e:
            rospy.logerr("Error in image callback: {e}".format(e))

if __name__ == '__main__':
    ocr_node = None
    try:
        ocr_node = OcrCompressedNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        ocr_node.s.close()
        rospy.loginfo("Socket closed.")
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV windows destroyed.")
