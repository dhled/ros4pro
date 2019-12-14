#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import imageio
import torch
import collections
from rospkg.rospack import RosPack
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

from os.path import join
from std_msgs.msg import UInt32, UInt8
from ros4pro.srv import VisionPredict, VisionPredictResponse
from ros4pro.vision.models import LeNet
from ros4pro.vision.detection import get_box_contours, get_sprites, preprocess_sprites

LABELS=[1, 2]
Box = collections.namedtuple('Box', 'contour sprite label')

rospack = RosPack()
path = rospack.get_path('ros4pro')
bridge = CvBridge()

def process(image, model, debug=None):
    """
    This function processes an image given a model, and returns a list of Box.
    """
    debug_inner = debug in ["inner", "all"]
    contours = get_box_contours(image, debug=debug_inner)
    sprites = get_sprites(image, contours, debug=debug_inner)
    inputs = preprocess_sprites(sprites, debug=debug_inner)
    labels = [model.infer(i) for i in inputs]

    boxes = [Box(contour=c, sprite=s, label=l) for c, s, l in  zip(contours, sprites, labels)]

    if debug in ["all", "synthesis"]:
        for box in boxes:
            fig, ax = plt.subplots(nrows=2)
            ax[0].imshow(image)
            ax[0].plot(box.contour[:,0], box.contour[:,1], "og")
            ax[0].plot(box.contour.mean(axis=0)[0], box.contour.mean(axis=0)[1], "og")
            ax[1].imshow(box.sprite)
            ax[1].set_title("Label recognized: {}".format(box.label))
            plt.show()

    return boxes


def handle_predict(request):
    image = bridge.imgmsg_to_cv2(request.image)
    debug = "synthesis" if rospy.get_param("/ros4pro/vision/debug", False) else None
    boxes = process(image, model, debug=debug)
    
    rospy.loginfo("Vision server found {} boxe(s) in this image".format(len(boxes)))

    response = VisionPredictResponse()
    for box in boxes:
        if len(box.contour) == 4:
            response.x1.append(UInt32(box.contour[0][0]))
            response.y1.append(UInt32(box.contour[0][1]))
            response.x2.append(UInt32(box.contour[1][0]))
            response.y2.append(UInt32(box.contour[1][1]))
            response.x3.append(UInt32(box.contour[2][0]))
            response.y3.append(UInt32(box.contour[2][1]))
            response.x4.append(UInt32(box.contour[3][0]))
            response.y4.append(UInt32(box.contour[3][1]))
            response.label.append(UInt32(box.label))
    return response

if __name__ == "__main__":
    rospy.init_node('vision_server')

    labels = [1, 2]
    checkpoint_path = join(path, "checkpoints", "checkpoint")

    model = LeNet(classes=labels)
    model.load_state_dict(torch.load(checkpoint_path))
    model.eval()

    service = rospy.Service('ros4pro/vision/predict', VisionPredict, handle_predict)
    rospy.loginfo("Vision server is ready, waiting for requests")
    rospy.spin()