#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
from motor_driver_package.msg import MotorSpeedRequest
from controller_package.msg import MovementRequest
import cv2
import numpy as np
from jetson_camera.msg import ProcessedImages
import os
import socket
import struct
import Queue
import threading
from levenshtein import levenshtein 
import math

ocr_server_ip = "192.168.8.169"

JIMMY_STATE = {
    "moving": 0,
    "idle": 1,
    "stopped": 2,
}

class CommandDecoder:
    AVAILABLE_COMMANDS = ["LEFT", "RIGHT", "FORWARD"]

    def __init__(self):
        pass

    def degrees_to_rad(self, deg):
        return deg * math.pi / 180

    def create_command_request(self, command):
        if command not in self.AVAILABLE_COMMANDS:
            rospy.logwarn("comman decoder: command {} not a valid command".format(command))
            return None
        
        msg = MovementRequest()

        if command is "LEFT":
            msg.request_type = 2 
            # FIXME: this might be problematic not sure about the value
            msg.value = self.degrees_to_rad(-90)
        elif command is "RIGHT":
            msg.request_type = 2 
            # FIXME: this might be problematic not sure about the value
            msg.value = self.degrees_to_rad(90)
        elif command is "FORWARD":
            msg.request_type = 1
            msg.value = 0.3
        else:
            rospy.warninf("messed up the switch statements bro")
            msg.request_type = 1
            msg.value = 1
        
        return msg
        
    
    
class CommandHistory:
    
    AVAILABLE_COMMANDS = ["LEFT", "RIGHT", "FORWARD"]

    def __init__(self):
        self.history = {cmd:0 for cmd in self.AVAILABLE_COMMANDS}
        rospy.loginfo("history {}".format(self.history))

    def add(self, command):
        if command not in self.AVAILABLE_COMMANDS:
            rospy.logwarn("unknown command {}".format(command))
            return
        
        self.history[command] = self.history[command] + 1
        rospy.loginfo("added 1 to command: {}. Current value: {}".format(command, self.history[command]))

        if self.history[command] > 10:
            return command
        else:
            return None
    
    def guess(self, potential_command):
        potential_command = potential_command.upper()
        
        for ac in self.AVAILABLE_COMMANDS:
            score = levenshtein(ac, potential_command) 
            # possible commands are completely different lexicographically we can be confident with 
            # distance 1
            if score <= 1:
                rospy.loginfo("guess: {guess}, match: {match}, score: {score}"
                              .format(guess=potential_command, match=ac, score=score))

                return self.add(ac)

        rospy.logwarn("guess {} did not match any command".format(potential_command
                                                                ))

    def reset(self):
        self.history = {cmd:0 for cmd in self.AVAILABLE_COMMANDS}
        

class OcrCompressedNode:
    def __init__(self):
        self.initialized = False 
        rospy.init_node('ocr_node', anonymous=True)
        rospy.loginfo("OCR Node (CompressedImage) initialized.")
        self.s = socket.socket()
        self.s.connect((ocr_server_ip, 9999)) 
        rospy.loginfo("Connected to socket server.")

        self.commands_history = CommandHistory()
        self.command_decoder = CommandDecoder()
        self.state = JIMMY_STATE["idle"]
        self.state_lock = threading.Lock()
        
        self.image_queue = Queue.Queue()
        self.response_queue = Queue.Queue(maxsize=2)
        
        self.sub_image = rospy.Subscriber(
            "/camera/image_processed_vlad",
            ProcessedImages,
            self.image_cb,
            buff_size=2**25,
            queue_size=1
        )
    
        self.sub_motors= rospy.Subscriber(
            "/motor_driver/motors",
            MotorSpeedRequest,
            self.motor_cb,
            #Change buff size and queue size accordingly
            buff_size=1000000,
            queue_size=1,
        )

        self.commands_pub = rospy.Publisher(
            "/motor_driver/commands",
            MovementRequest,
            queue_size=1,
        )

        send_t = threading.Thread(target=self.send_images)
        rec_t = threading.Thread(target=self.receive_text)
        motor_t = threading.Thread(target=self.watch_history)
        send_t.daemon = True
        rec_t.daemon = True
        motor_t.daemon = True
        send_t.start()
        rec_t.start()
        motor_t.start()

        self.initialized = True

        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()
    
    
    def watch_history(self):
        rospy.loginfo("in watch history")
        consensus = None
        while not rospy.is_shutdown():
            possible_command = self.response_queue.get()
            rospy.loginfo("received possible command {}".format(possible_command))
            
            self.state_lock.acquire()
            if self.state == JIMMY_STATE["stopped"]:
                self.state = JIMMY_STATE["idle"]
                self.commands_history.reset()

            if self.state == JIMMY_STATE["idle"]:
                consensus = self.commands_history.guess(possible_command)
                rospy.loginfo("reached consesnus command: {}".format(consensus))
            
            self.state_lock.release()
            
            if consensus is not None:
                comm = self.command_decoder.create_command_request(consensus)
                rospy.loginfo("command request: {}".format(comm))
                self.commands_pub.publish(comm)
                self.commands_history.reset()
                rospy.loginfo("will sleep now... zzzzz")
                rospy.sleep(5.)
        
    
    def motor_cb(self, motor_data):
        if not self.initialized:
            return

        rospy.loginfo("Received request: left_wheel = %f, right_wheel = %f",
            motor_data.speed_left_wheel, motor_data.speed_right_wheel)
        
        self.state_lock.acquire()
        if motor_data.speed_left_wheel == 0 and motor_data.speed_right_wheel == 0:
            if self.state != JIMMY_STATE["idle"]:
                # watch_history func will change this to idle 
                self.state = JIMMY_STATE["stopped"]
        else:
            self.state = JIMMY_STATE["moving"]

        self.state_lock.release()

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
            rospy.loginfo("Socket error: {e}".format(e))


    def image_cb(self, msg):
        if not self.initialized:
            return

        # Convert image to OpenCV format
        # np_arr = np.fromstring(msg.image.data, np.uint8)
        # img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_data = msg.undistorted_image.data
        self.image_queue.put(image_data)

    def receive_text(self):
        print("in receive text")
        while not rospy.is_shutdown():
            raw_len = self.recvall(4)
            if not raw_len:
                continue
            text_len = struct.unpack('>I', raw_len)[0]
            text_data = self.recvall(text_len)
            detected_text = text_data.decode('utf-8')
            rospy.loginfo("detected text {}".format(detected_text))
            # self.pub_text.publish(detected_text)

            try:
                self.response_queue.put_nowait(detected_text) 
            except Queue.Full:
                rospy.loginfo("queue is full")
                pass

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
        rospy.loginfo("OCR Node (CompressedImage) initialized.")
        # while not rospy.is_shutdown():
    except rospy.ROSInterruptException:
        pass
    finally:
        # ocr_node.s.close()
        rospy.loginfo("Socket closed.")
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV windows destroyed.")
