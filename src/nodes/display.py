#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import cv2, cv_bridge
from os.path import join
from getpass import getuser
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HeadDisplay(object):
    def __init__(self, width, height, font=cv2.FONT_HERSHEY_SIMPLEX, scale=1, thickness=1, color=[255]*3, interline=1.1):
        self.rospack = rospkg.RosPack()
        self.bridge = CvBridge()
        self.font = font
        self.scale = scale
        self.thickness = thickness
        self.color = color
        self.publisher = rospy.Publisher("/robot/head_display", Image, queue_size=1)
        self.width, self.height = width, height
        self.image = cv2.imread(join(self.rospack.get_path("ros4pro"), "assets", "background.png"))

    def blit_text(self, image, text, font=cv2.FONT_HERSHEY_SIMPLEX, scale=2.3, thickness=5, color=[255]*3):
        blit_image = image.copy()
        def horizontal_align(sentence):
            (width, height), _ = cv2.getTextSize(sentence, font, scale, thickness)
            return width
        x, y = self.width/2 - horizontal_align(text)/2, self.height/2 + 150
        cv2.putText(blit_image, text, (x, y), font, scale, color, thickness=thickness)
        return blit_image

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            blit_image = self.blit_text(self.image, getuser().upper())
            ros_image = self.bridge.cv2_to_imgmsg(blit_image, encoding="bgr8")
            self.publisher.publish(ros_image)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('head_display')
    head = HeadDisplay(1024, 600, font=cv2.FONT_HERSHEY_SIMPLEX, scale=3, thickness=1)
    head.run()