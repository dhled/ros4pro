#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import traceback
import time
from std_srvs.srv import Empty
from std_msgs.msg import String
import numpy as np


from sensor_msgs.msg import LaserScan, PointCloud2, Image
from geometry_msgs.msg import Twist
import std_msgs.msg
from laser_geometry import LaserProjection

import tf
import traceback
from catie_utils import pid


class StartUtils:
    def __init__(self):
        try:
            # For clearing the costmaps
            self.reset_costmaps_service = rospy.ServiceProxy(
                "/move_base/clear_costmaps", Empty
            )
            self.pub_cmd = rospy.Publisher(
                "/mobile_base_controller/cmd_vel", Twist, queue_size=10
            )
            self.t0 = time.time()
        except Exception:
            rospy.logerr(traceback.format_exc())
            rospy.logerr("Failed to init Start_utils")

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _publish_speed(self, linear_speed, angular_speed):
        twist = self._get_twist(linear_speed, angular_speed)
        # rospy.loginfo(twist)
        self.pub_cmd.publish(twist)

    def advanced_door_passing(
        self, linear_speed, angular_speed, full_speed_duration, acceleration_duration=2
    ):
        # Dark times
        period = 0.01
        # Accelerating phase
        t0 = time.time()
        t = t0
        while t - t0 < acceleration_duration:
            smoothing = (t - t0) / float(acceleration_duration)
            lin = linear_speed * smoothing
            ang = angular_speed * smoothing
            self._publish_speed(lin, ang)
            t = time.time()
            rospy.sleep(period)

        # Cruiser phase
        t0 = time.time()
        t = t0
        while t - t0 < full_speed_duration:
            self._publish_speed(linear_speed, angular_speed)
            t = time.time()
            rospy.sleep(period)

        # Deccelerating phase
        t0 = time.time()
        t = t0
        while t - t0 < acceleration_duration:
            smoothing = (t - t0) / float(acceleration_duration)
            lin = max(0, linear_speed * (1 - smoothing))
            ang = max(0, angular_speed * (1 - smoothing))
            self._publish_speed(lin, ang)
            t = time.time()
            rospy.sleep(period)

        # For good measure
        self._publish_speed(0, 0)
        rospy.sleep(period)
        self._publish_speed(0, 0)

    def reset_t0(self):
        self.t0 = time.time()

    def reset_costmaps(self, wait_afterwards=3.0):
        self.reset_costmaps_service()
        rospy.sleep(wait_afterwards)

    def is_start_test(self, delay_at_start=0.0, timeout=1.0):
        t = time.time()
        if (t - self.t0) < delay_at_start:
            return False
        try:
            rospy.wait_for_message("/start_test", String, timeout)
        except Exception:
            rospy.logerr(traceback.format_exc())
            return False
        return True

    def advanced_laser_to_is_door_open(
        self, min_dist, max_dist, angle, is_nan_open=False
    ):
        """Listen to the /scan LaserScan topic and returns True if the door is open
        angle==0 means in front of the robot
        """
        # Real
        # angle_min: -1.56505846977
        # angle_max: 1.56458568573
        # angle_increment: 0.00581718236208
        laser_scan = rospy.wait_for_message("/scan", LaserScan, timeout=None)
        angle_increment = laser_scan.angle_increment
        angle_min = laser_scan.angle_min
        index_middle = len(laser_scan.ranges) / 2
        index = int(index_middle + (angle / abs(angle_min)) * index_middle)
        index = max(0, index)
        index = min(len(laser_scan.ranges) - 1, index)

        r = laser_scan.ranges[index]
        rospy.loginfo("r = {}".format(r))
        try:
            d = float(r)
            if np.isnan(d):
                # Opposite behaviour because the door might be so close we don't see it
                return is_nan_open
            if d >= min_dist and d <= max_dist:
                return True
            return False
        except:
            # inf?
            return False
