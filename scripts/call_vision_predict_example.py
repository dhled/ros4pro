#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This script tests the vision server with a static picture from a JPG file:
#   1. Start the vision server: rosrun ros4pro vision_server.py
#   2. Change the file path to the JPG image hereunder (line 10)
#   3. Call this script: rosrun ros4pro call_vision_predict_example.py
#   4. If a box is found in your JPG image, the detected label 1 or 2 will be printed as well as the coordinates of its 4 corners

PATH = "/home/arbalet/Téléchargements/cubes/straight/2_4.jpg"

from ros4pro.srv import VisionPredict, VisionPredictRequest
import cv2, rospy
from cv_bridge import CvBridge

bridge = CvBridge()
image = cv2.imread(PATH)
greyscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
image_msg = bridge.cv2_to_imgmsg(greyscale_image)
predict = rospy.ServiceProxy('ros4pro/predict', VisionPredict)
response = predict.call(VisionPredictRequest(image=image_msg))

print(response)