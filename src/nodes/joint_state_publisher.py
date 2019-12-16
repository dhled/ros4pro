#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
from sensor_msgs.msg import JointState
from copy import deepcopy
from threading import RLock

class JointStatePublisher(object):
    RATE_HZ=20
    def __init__(self):
        self._lock = RLock()
        self._js = JointState()
        self._js.name = ["right_j" + str(j) for j in range(7)] + ["head_pan", "right_gripper_l_finger_joint"]
        self._js.position = [0]*len(self._js.name)
        rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, self._cb_fake_js)
        self._pub_js = rospy.Publisher("/robot/joint_states", JointState, queue_size=1)

    def _cb_fake_js(self, msg):
        with self._lock:
            self._js = msg

    def run(self):
        rate = rospy.Rate(self.RATE_HZ)
        js = deepcopy(self._js)
        while not rospy.is_shutdown():
            try:
                js.header.stamp = rospy.Time.now()
                with self._lock:
                    for j, joint in enumerate(js.name):
                        if joint in self._js.name:
                            i = self._js.name.index(joint)
                            js.position[i] = self._js.position[j]
                    self._pub_js.publish(js)
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                return

if __name__=="__main__":
    rospy.init_node("joint_state_publisher")
    JointStatePublisher().run()