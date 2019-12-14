#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This script tests the vision server with a static picture from a JPG file:
#   1. Start the vision server: rosrun ros4pro vision_server.py
#   2. Change the file path to the JPG image hereunder (line 10) or use None to get the current image from Sawyer
#   3. Call this script: rosrun ros4pro call_vision_predict_example.py
#   4. If a box is found in your JPG image, the detected label 1 or 2 will be printed as well as the coordinates of its 4 corners

PATH = "/home/arbalet/Téléchargements/cubes/straight/2_4.jpg"
PATH = None

from ros4pro.srv import VisionPredict, VisionPredictRequest
import cv2, rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("call_vision_predict_example")
image_msg = None

def _cb_image(msg):
    global image_msg
    image_msg = msg

if PATH:
    image = cv2.imread(PATH)
    greyscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_msg = bridge.cv2_to_imgmsg(greyscale_image)
else:
    rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, _cb_image)
    rospy.sleep(0.5)

bridge = CvBridge()
predict = rospy.ServiceProxy('ros4pro/vision/predict', VisionPredict)
response = predict.call(VisionPredictRequest(image=image_msg))

print(response)