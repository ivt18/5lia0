#!/usr/bin/env python2

import cv2
import numpy as np
import darknet

# Load YOLO model
net, class_names, class_colors = darknet.load_network(
    "cfg/yolov4-tiny.cfg",
    "cfg/coco.data",
    "yolov4-tiny.weights",
    batch_size=1)

# Load image using OpenCV
image_bgr = cv2.imread("data/gustavo.jpg")
image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
height, width, _ = image_rgb.shape

# Resize image to network input size
network_width = darknet.network_width(net)
network_height = darknet.network_height(net)
image_resized = cv2.resize(image_rgb, (network_width, network_height))

# Convert image to Darknet format
darknet_image = darknet.make_image(network_width, network_height, 3)
darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())

# Perform detection
detections = darknet.detect_image(net, class_names, darknet_image)

# Print results
print(detections)

# Free Darknet image memory
darknet.free_image(darknet_image)

# Draw detections on image
for label, confidence, bbox in detections:
    x, y, w, h = map(int, bbox)
    left = int(x - w / 2)
    top = int(y - h / 2)
    right = int(x + w / 2)
    bottom = int(y + h / 2)

    cv2.rectangle(image_resized, (left, top), (right, bottom), class_colors[label], 2)
    cv2.putText(image_resized, "{} [{}]".format(label, confidence), (left, top - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, class_colors[label], 2)

# Convert back to BGR for OpenCV display
image_bgr = cv2.cvtColor(image_resized, cv2.COLOR_RGB2BGR)
cv2.imshow("Detections", image_bgr)
cv2.waitKey(0)
cv2.destroyAllWindows()
