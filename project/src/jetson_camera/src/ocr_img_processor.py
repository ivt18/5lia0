#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
# from motor_driver_package.msg import MotorSpeedRequest
# from controller_package.msg import MovementRequest
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
from std_msgs.msg import Bool

# TODO: make this a parameter/config file entry
ocr_server_ip = "192.168.0.48"
consensus_threshold = 3  # how many times a command must be seen before it is considered valid

class CommandDecoder:
    AVAILABLE_COMMANDS = ["STOP"]

    def __init__(self):
        pass

    def degrees_to_rad(self, deg):
        return deg * math.pi / 180

    def create_command_request(self, command):
        if command != None and command not in self.AVAILABLE_COMMANDS:
            rospy.logwarn("comman decoder: command {} not a valid command".format(command))
            return None

        msg = Bool()

        if command == "STOP":
            # False: stop sign detected, otherwise true
            msg.data = False
        else:
            msg.data = True

        return msg


class CommandHistory:

    AVAILABLE_COMMANDS = ["STOP"]

    def __init__(self):
        self.history = {cmd:0 for cmd in self.AVAILABLE_COMMANDS}
        rospy.loginfo("history {}".format(self.history))

    def add(self, command):
        if command not in self.AVAILABLE_COMMANDS:
            rospy.logwarn("unknown command {}".format(command))
            return

        self.history[command] = self.history[command] + 1
        # rospy.loginfo("added 1 to command: {}. Current value: {}".format(command, self.history[command]))

        if self.history[command] > consensus_threshold:
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
                # rospy.loginfo("guess: {guess}, match: {match}, score: {score}"
                              # .format(guess=potential_command, match=ac, score=score))

                return self.add(ac)

        rospy.logerr("guess {} did not match any command".format(potential_command
                                                                ))

    def reset(self):
        self.history = {cmd:0 for cmd in self.AVAILABLE_COMMANDS}


class OcrCompressedNode:
    def __init__(self):
        self.initialized = False
        rospy.init_node('ocr_node', anonymous=True)
        rospy.loginfo("OCR Node (CompressedImage) initialized.")

        self.s = self.setup_connection()
        self.message_rate = rospy.Rate(10)

        self.commands_history = CommandHistory()
        self.command_decoder = CommandDecoder()

        self.image_queue = Queue.Queue()
        self.response_queue = Queue.Queue(maxsize=2)

        self.sub_image = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.image_cb,
            buff_size=2**25,
            queue_size=1
        )

        self.commands_pub = rospy.Publisher(
            "/camera/text",
            Bool,
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

    # TODO: make oocr server configurable from roslaunch
    def setup_connection(self, max_retries = None):
        retries = 0
        while not rospy.is_shutdown():
            try:
                s = socket.socket()
                s.connect((ocr_server_ip, 9999))
                rospy.loginfo("Connected to OCR server.")

                return s
            except socket.error as e:
                rospy.logerr("Connection to OCR serverfailed: {}".format(e))
                retries += 1
                if max_retries is not None and retries >= max_retries:
                    rospy.signal_shutdown("Failed to connect to server after {} retries.".format(retries))
                    raise Exception("Max retries reached. Server not available.")

                rospy.sleep(rospy.Duration(5))

    def watch_history(self):
        rospy.loginfo("in watch history")
        consensus = None
        while not rospy.is_shutdown():
            possible_command = self.response_queue.get()
            # rospy.loginfo("received possible command {}".format(possible_command))

            consensus = self.commands_history.guess(possible_command)


            comm = self.command_decoder.create_command_request(consensus)
            # rospy.loginfo("command request: {}".format(comm))
            self.commands_pub.publish(comm)

            if consensus is not None:
                rospy.logwarn("reached consesnus command: {}; will publish".format(consensus))
                self.commands_history.reset()

            self.message_rate.sleep()


    def shutdown_hook(self):
        rospy.loginfo("Shutting down OCR node.")
        try:
            self.s.close()
        except:
            pass

    def send_images(self):
        print("sending images func")
        # TODO: can move connection setup here
        while not rospy.is_shutdown():
            img = self.image_queue.get()
            img_len = struct.pack('>I', len(img))
            self.s.sendall(img_len + img)

        print("sending images func cleanup")

    def image_cb(self, msg):
        if not self.initialized:
            return

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
            # rospy.loginfo("detected text {}".format(detected_text))
            # self.pub_text.publish(detected_text)

            try:
                self.response_queue.put_nowait(detected_text)
            except Queue.Full:
                rospy.loginfo("[OCR] ocr detection queue is full")
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
