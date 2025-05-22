#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from jetson_camera.msg import ProcessedImages
import os
import socket
import struct
import Queue
import threading

ocr_server_ip = "192.168.8.169"

class OcrCompressedNode:
    def __init__(self):
        rospy.init_node('ocr_node', anonymous=True)
        rospy.loginfo("OCR Node (CompressedImage) initialized.")
        self.s = socket.socket()
        self.s.connect((ocr_server_ip, 9999)) 
        rospy.loginfo("Connected to socket server.")
        
        self.image_queue = Queue.Queue()
        self.response_queue = Queue.Queue()
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed_vlad",
            ProcessedImages,
            self.image_cb,
            buff_size=2**25,
            queue_size=1
        )

        threading.Thread(target=self.send_images, daemon=True).start()
        threading.Thread(target=self.receive_text, daemon=True).start()
        rospy.on_shutdown(self.shutdown_hook)
        # TODO: uncomment this when done
    
    def shutdown_hook(self):
        rospy.loginfo("Shutting down OCR node.")
        try:
            self.s.close()
        except:
            pass

    def send_images(self):
        print("sending images func")
        while not rospy.is_shutdown():
            img = self.image_queue.get()
            img_len = struct.pack('>I', len(img))
            self.s.sendall(img_len + img)

        print("sending images func cleanup")

    def send_image_over_socket(self, image_data):
        # rospy.loginfo("Sending image over socket...")
        try:
            length = struct.pack('>I', len(image_data))
            self.s.sendall(length + image_data)
        except Exception as e:
            rospy.logerr("Socket error: {e}".format(e))


    def image_cb(self, msg):
        # Convert image to OpenCV format
        # np_arr = np.fromstring(msg.image.data, np.uint8)
        # img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_data = msg.undistorted_image.data
        self.image_queue.put(image_data)

    # def image_cb(self, msg):
    #     # rospy.Rate(10).sleep()  
    #     image_data = msg.undistorted_image.data
    #     rospy.loginfo("Received image data of length: {}".format(len(image_data)))
    #     try:
    #         np_arr = np.frombuffer(msg.raw_image.data, np.uint8)
    #         image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #         # self.extract_text(cv2.imdecode(np.frombuffer(msg.undistorted_image.data, np.uint8), cv2.IMREAD_COLOR))
    #         self.send_image_over_socket(image_data)
    #         raw_len = recvall(self.s, 4)
    #         text_len = struct.unpack('>I', raw_len)[0]
    #         # rospy.loginfo(f"Text length: {text_len}")
    #         text_data = recvall(self.s, text_len)
    #         detected_text = text_data.decode('utf-8')
    #         rospy.loginfo("Detected text: {detected_text}".format(detected_text=detected_text)) 

    #         # cv2.imshow("Raw Image", image)
    #         # cv2.waitKey(1)  

    #         # text = pytesseract.image_to_string(image)

    #         # rospy.loginfo(f"OCR Text: {text.strip()}")

        # except Exception as e:
        #     rospy.logerr("Error in image callback: {}".format(e))
            
    def receive_text(self):
        print("in receive text")
        while not rospy.is_shutdown():
            raw_len = self.recvall(4)
            if not raw_len:
                continue
            text_len = struct.unpack('>I', raw_len)[0]
            text_data = self.recvall(text_len)
            detected_text = text_data.decode('utf-8')
            rospy.logger("detected text {}".format(detected_text))
            # self.pub_text.publish(detected_text)

        print("reeive text cleanup")

    def recvall(self, length):
        data = b''
        while len(data) < length:
            more = self.s.recv(length - len(data))
            if not more:
                raise EOFError("Socket closed before receiving expected data")
            data += more
        return data

if __name__ == '__main__':
    ocr_node = None
    try:
        ocr_node = OcrCompressedNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        ocr_node.s.close()
        rospy.loginfo("Socket closed.")
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV windows destroyed.")
