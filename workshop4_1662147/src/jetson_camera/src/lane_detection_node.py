#!/usr/bin/env python2

import math

import cv2 as cv
import numpy as np
import rospy

from cv_bridge import CvBridgeError

import datatypes

from controller_package.msg import MovementRequest
from jetson_camera.msg import ProcessedImages


class LaneDetectionNode:

    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing {node_name}...".format(node_name=node_name))
        rospy.init_node(self.node_name, anonymous=True)

        # construct camera subscriber
        self.subscriber = rospy.Subscriber(
            "/camera/image_processed",
            ProcessedImages,
            self.receive_img,
            buff_size = 2**24,
            queue_size = 1
        )

        # construct lanes publisher
        self.publisher = rospy.Publisher(
            "/motor_driver/commands",
            MovementRequest,
            queue_size=1,
        )

        # video settings
        fourcc = cv.VideoWriter_fourcc(*'acv1')
        self.video = cv.VideoWriter(
            "/home/jetbot/EVC/workshops/workshop2_1662147/lane_detection.avi",
            fourcc,
            15.0,
            (640, 480)
        )

        self.first_image_received = False
        self.initialized = True
        rospy.loginfo("{node_name} initialized".format(node_name=node_name))

    def receive_img(self, data):
        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("{node_name} received the first image".format(node_name=self.node_name))

        try:
            # decode image without CvBridge
            img = cv.imdecode(np.frombuffer(data.undistorted_image.data, np.uint8), cv.IMREAD_COLOR)
            width, height = img.shape[1], img.shape[0]
            # rimg = cv.resize(img, (600, 480))
            cv.imshow("Input", img)

            # highlight and extract edges from image
            gray_img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
            cannyed_img = cv.Canny(gray_img, 100, 200)
            cannyed_img = cv.resize(cannyed_img, (600, 480))

            # cv.imshow("Canny", cannyed_img)

            # crop image to ROI
            roi_vertices = [(0, height), (width / 2, height / 2), (width, height)]
            roi_img = self.region_of_interest(cannyed_img, np.array([roi_vertices], np.int32))

            # roi_img = roi_img.resize(roi_img, (600, 480))
            # cv.imshow("ROI", roi_img)

            # apply Hough line transformation
            hough_lines = cv.HoughLinesP(roi_img,
                rho=6,
                theta = np.pi / 60,
                threshold = 160,
                lines = np.array([]),
                minLineLength=40,
                maxLineGap = 25
            )
            
            # split left and right lines based on slope
            lines = self.split_lines(hough_lines)
            if lines is not None:
                n_left = len(lines['left']['x'])
                n_right = len(lines['right']['x'])
                n = n_left + n_right
                # rospy.loginfo("detected {n} lines".format(n=n))
            # else:
                # rospy.loginfo("detected 0 lines")

            # fit a linear polynomial to the left and right lines
            min_y = int(img.shape[0] * 3 / 5)   # slightly below the middle of the image
            # min_y = int(img.shape[0] * 1 / 3)   # slightly below the middle of the image
            max_y = img.shape[0]              # bottom of the image
            poly = {}
            start = {'left': {}, 'right': {}}
            end = {'left': {}, 'right': {}}
            
            for side in ['left', 'right']:
                if lines[side]['x'] and lines[side]['y']:
                    # rospy.loginfo("detected lane")
                    poly[side] = np.poly1d(np.polyfit(lines[side]['y'], lines[side]['x'], deg=1))
                    start[side]['x'] = int(poly[side](max_y))
                    end[side]['x'] = int(poly[side](min_y))
                else:
                    # default if no line detected
                    start[side]['x'] = 0
                    end[side]['x'] = 0

            # publish lanes
            self.publish(start, end)
            
            # create filled polygon between the lanes
            lane_img = self.draw_lane_lines(
                img,
                [start['left']['x'], max_y, end['left']['x'], min_y],
                [start['right']['x'], max_y, end['right']['x'], min_y]
            )

            # record video
            # vid = cv.resize(lane_img, (640, 480))
            # self.video.write(vid)

            # display lane_img
            lane_img = cv.resize(lane_img, (600, 480))
            cv.imshow("Lane Detection", lane_img)
            cv.waitKey(1)

        except CvBridgeError as err:
            rospy.logerr("Error converting image: {err}".format(err=err))

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img)
        match_mask_color = 255
        cv.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv.bitwise_and(img, mask)
        return masked_image
    
    def split_lines(self, h_lines):
        lines = {'left': {'x': [], 'y': []}, 'right': {'x': [], 'y': []}}
        if h_lines is None:
            return lines

        for line in h_lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
                
                # ignore nearly horizontal lines
                if math.fabs(slope) < 0.5:
                    continue
                
                side = 'left' if slope <= 0 else 'right'
                lines[side]['x'].extend([x1, x2])
                lines[side]['y'].extend([y1, y2])

        return lines

    def publish(self, start, end):
        msg = MovementRequest()

        msg.request_type = datatypes.MovementRequest.MOVEMENT_REQUEST

        # lane_detected -> move forward
        if (start['left']['x'] != 0) and (start['right']['x'] != 0) and \
            (end['left']['x'] != 0) and (end['right']['x'] != 0):
            msg.value = 0.1
            rospy.loginfo("Published request: {typ}/{val}".format(typ=msg.request_type, val=msg.value))
            self.publisher.publish(msg)
    
    def draw_lane_lines(self, image, left_lane, right_lane, color=[0, 255, 0], thickness=10):
        lane_img = np.zeros_like(image)
        poly_points = np.array([[
            (left_lane[0], left_lane[1]),
            (left_lane[2], left_lane[3]),
            (right_lane[2], right_lane[3]),
            (right_lane[0], right_lane[1])
        ]], dtype=np.int32)

        # fill polygon between the lanes
        cv.fillPoly(lane_img, poly_points, color)

        # overlay the polygon over the original image
        return cv.addWeighted(image, 0.8, lane_img, 0.5, 0.0)

if __name__ == '__main__':
    # initialize node
    rospy.init_node('lane_detection_node', anonymous=True)
    lane_detection_node = LaneDetectionNode('lane_detection_node')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down lane detection node")
    finally:
        cv.destroyAllWindows()
        lane_detection_node.video.release()

