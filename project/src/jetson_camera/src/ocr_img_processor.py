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
from collections import deque, Counter
import time
import select

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
    UNKNOWN_COMMAND = "UNKNOWN"

    def __init__(self):
        # self.history = {cmd:0 for cmd in self.AVAILABLE_COMMANDS}
        # circular buffer of the last 7 commands
        # add with append(), remove with popleft()
        self.history = deque(maxlen=7)
        rospy.loginfo("history {}".format(self.history))
    

    def majority_count(self):
        """
        Unless the history is empty, which happens only on the first call, return the most common command in the history.
        """
        c = Counter(self.history)

        if len(c) == 0:
            return None

        return c.most_common(1)[0][0]


    def add(self, command):
        if command not in self.AVAILABLE_COMMANDS and command != self.UNKNOWN_COMMAND:
            # this should really noy happen
            rospy.logerr("[COMMAND HISTORY] unknown command {}".format(command))
            return

        # self.history[command] = self.history[command] + 1
        # rospy.loginfo("added 1 to command: {}. Current value: {}".format(command, self.history[command]))
        self.history.append(command)

        # if self.history[command] > consensus_threshold:
        #     return command
        # else:
        #     return None


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

        return self.add(self.UNKNOWN_COMMAND)

    def reset(self):
        # self.history = {cmd:0 for cmd in self.AVAILABLE_COMMANDS}
        self.history.clear()


class OcrCompressedNode:
    def __init__(self, ocr_server_address):
        self.initialized = False
        # a tuple (ip, port) for the OCR server
        self.ocr_server_address = ocr_server_address
        self.connected = False
        self.reconnect_delay = 2 
        self.connection_lock = threading.Lock()
        
        rospy.init_node('ocr_node', anonymous=True)
        rospy.loginfo("OCR Node (CompressedImage) initialized.")

        self.s = self.setup_connection()

        if not self.s:
            rospy.logerr("Failed to connect to OCR server. Shutting down node.")
            rospy.signal_shutdown("Failed to connect to OCR server.")
            return

        self.message_rate = rospy.Rate(1)

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

    def setup_connection(self, max_retries = None):
        retries = 0
        while not rospy.is_shutdown():
            try:
                with self.connection_lock:
                    if hasattr(self, 's') and self.s:
                        try:
                            self.s.close()
                        except:
                            pass
                    
                    s = socket.socket()
                    s.settimeout(10)  
                    s.connect(self.ocr_server_address)
                    self.connected = True
                    self.reconnect_delay = 1  
                    rospy.loginfo("Connected to OCR server.")
                    return s
                    
            except socket.error as e:
                rospy.logerr("Connection to OCR server failed: {}".format(e))
                self.connected = False
                retries += 1
                if max_retries is not None and retries >= max_retries:
                    rospy.signal_shutdown("Failed to connect to server after {} retries.".format(retries))
                    raise Exception("Max retries reached. Server not available.")

                # delay = min(self.reconnect_delay, self.max_reconnect_delay)
                rospy.loginfo("Retrying connection in {} seconds...".format(self.reconnect_delay))
                rospy.sleep(rospy.Duration(self.reconnect_delay))
                # self.reconnect_delay = min(self.reconnect_delay * 2, self.max_reconnect_delay)

    def reconnect(self):
        """Attempt to reconnect to the OCR server"""
        rospy.logwarn("Attempting to reconnect to OCR server...")
        try:
            self.s = self.setup_connection()
            if self.s:
                rospy.loginfo("Successfully reconnected to OCR server.")
                return True
        except Exception as e:
            rospy.logerr("Reconnection failed: {}".format(e))
        return False

    def is_connected(self):
        """Check if the socket is still connected"""
        if not self.s:
            return False
        
        try:
            ready_to_read, ready_to_write, in_error = select.select(
                            [self.s],
                            [],
                            [self.s],
                            5)
            
            if self.s in ready_to_read:
                return True 
            
            rospy.logwarn("OCR client socket not ready to read, assuming disconnected.")
            return True
        except socket.error:
            return False

    def watch_history(self):
        rospy.loginfo("in watch history")
        consensus = None
        while not rospy.is_shutdown():
            consensus = None
            try:
                possible_command = self.response_queue.get_nowait()
                self.commands_history.guess(possible_command)
                consensus = self.commands_history.majority_count()

            except Queue.Empty:
                pass

            comm = self.command_decoder.create_command_request(consensus)
            rospy.loginfo("command request: {}".format(comm))
            self.commands_pub.publish(comm)
            rospy.logwarn("reached consesnus command: {}; will publish".format(consensus))


            # if consensus is None:
            #     rospy.loginfo("no consensus reached yet")
            #     continue
            self.message_rate.sleep()


    def shutdown_hook(self):
        rospy.loginfo("Shutting down OCR node.")
        try:
            self.s.close()
        except:
            pass

    def send_images(self):
        print("sending images func")
        while not rospy.is_shutdown():
            try:
                img = self.image_queue.get(timeout=1)  # Add timeout to prevent blocking
                
                # Check connection before sending
                if not self.connected or not self.is_connected():
                    rospy.logwarn("Connection lost. Attempting to reconnect...")
                    if not self.reconnect():
                        continue
                
                with self.connection_lock:
                    img_len = struct.pack('>I', len(img))
                    self.s.sendall(img_len + img)
                    
            except Queue.Empty:
                continue
            except (socket.error, struct.error, EOFError) as e:
                rospy.logerr("Error sending image: {}".format(e))
                self.connected = False
                if not self.reconnect():
                    rospy.sleep(1) 
            except Exception as e:
                rospy.logerr("Unexpected error in send_images: {}".format(e))

        print("sending images func cleanup")

    def image_cb(self, msg):
        if not self.initialized:
            return

        image_data = msg.undistorted_image.data
        self.image_queue.put(image_data)

    def receive_text(self):
        print("in receive text")

        while not rospy.is_shutdown():
            try:
                # Check connection before receiving
                if not self.connected:
                    rospy.sleep(1)
                    continue
                
                with self.connection_lock:
                    raw_len = self.recvall(4)
                    if not raw_len:
                        continue
                    text_len = struct.unpack('>I', raw_len)[0]
                    text_data = self.recvall(text_len)
                    
                detected_text = text_data.decode('utf-8')
                
                try:
                    self.response_queue.put_nowait(detected_text)
                except Queue.Full:
                    rospy.loginfo("[OCR] ocr detection queue is full")
                    pass
                    
            except (socket.error, struct.error, EOFError) as e:
                rospy.logerr("Error receiving text: {}".format(e))
                self.connected = False
                if not self.reconnect():
                    rospy.sleep(1) 
            except Exception as e:
                rospy.logerr("Unexpected error in receive_text: {}".format(e))
                rospy.sleep(1)

        print("receive text cleanup")

    def recvall(self, length):
        data = b''
        while len(data) < length:
            if not self.connected:
                raise EOFError("Connection lost during receive")
                
            more = self.s.recv(length - len(data))
            if not more:
                self.connected = False
                raise EOFError("Socket closed before receiving expected data")
            data += more
        return data

if __name__ == '__main__':
    ocr_node = None
    ocr_server_ip = rospy.get_param("ocr_server_ip", "")
    ocr_server_port = rospy.get_param("ocr_server_port", 9999)
    print("Loaded OCR node parameters: IP = {}, Port = {}"
                  .format(
                      ocr_server_ip,
                      ocr_server_port
                      ))

    if ocr_server_ip == "":
        print("OCR server IP not set. Please set the parameter 'ocr_server_ip' in the launch file or parameter server.")
        raise ValueError("OCR server IP not set.")

    try:
        ocr_node = OcrCompressedNode((ocr_server_ip, ocr_server_port))
        rospy.loginfo("OCR Node (CompressedImage) initialized.")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        # ocr_node.s.close()
        rospy.loginfo("Socket closed.")
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV windows destroyed.")
