#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
from tf import TransformBroadcaster
from moveit_commander.conversions import list_to_pose, list_to_pose_stamped
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
from intera_interface.gripper import Gripper
from intera_interface.camera import Cameras

list_to_pose2 = lambda x: list_to_pose(x[0] + x[1])
list_to_pose_stamped2 = lambda x: list_to_pose_stamped(x[0] + x[1], "base")

class ManipulateNode(object):
    MIN_FRACTION = 0.8   # Minimum percentage of points successfully computed to initiate a cartesian trajectory
    def __init__(self):
        rospy.init_node("manipulate_sawyer")
        self.tfb = TransformBroadcaster()
        self.gripper = Gripper()
        self.commander = MoveGroupCommander("right_arm")
        self.camera = Cameras()
        self.scene = PlanningSceneInterface()
        with open("/home/arbalet/catkin_ws/src/ros4pro/config/pic.json") as f:
            self.config = json.load(f)

    def grasp(self, pose_grasp, z_approach_distance=0.18):
        self.gripper.open()

        # Go to approach pose
        pose_approach = [[pose_grasp[0][0], pose_grasp[0][1], pose_grasp[0][2] + z_approach_distance], pose_grasp[1]]      
        self.commander.set_pose_target(pose_approach[0] + pose_approach[1])
        success = self.commander.go()
        if not success:
            rospy.logerr("Can't find a valid path to approach pose")
            return False

        # Go to grasp pose
        grasp, fraction = self.commander.compute_cartesian_path(map(list_to_pose2, [pose_approach, pose_grasp]), 0.001, 5)
        self.tfb.sendTransform(pose_grasp[0], pose_grasp[1], rospy.Time.now(), "grasp", "base")
        
        if fraction < self.MIN_FRACTION:
            rospy.logerr("Can't compute a valid path to grasp")
            return False

        rospy.loginfo("{}% of success".format(int(fraction*100)))
        self.commander.execute(grasp)
        self.gripper.close()

        # Go to retreat
        retreat, fraction = self.commander.compute_cartesian_path(map(list_to_pose2, [pose_grasp, pose_approach]), 0.001, 5)

        if fraction < self.MIN_FRACTION:
            rospy.logerr("Can't compute a valid path to release")
            return False

        rospy.loginfo("{}% of success".format(int(fraction*100)))
        self.commander.execute(retreat)

        # Check if object has been grasped
        rospy.sleep(1)
        if not self.gripper.is_gripping():
            rospy.loginfo("Object hasn't been grasped, releasing")
            self.gripper.open()
            return False
        return True

    def place(self, pose_place):
        # Go to approach pose
        self.commander.set_pose_target(pose_place[0] + pose_place[1])
        success = self.commander.go()
        if not success:
            rospy.logerr("Can't find a valid path to approach pose")
            return False

        self.gripper.open()
        return True

    def run(self):
        rospy.sleep(1)
        self.scene.add_box("ground", list_to_pose_stamped2([[0, 0, 0], [0, 0, 0, 1]]), (0.65, 0.80, 0.001))
        self.scene.add_box("feeder", list_to_pose_stamped2([[-0.2, 0.57, 0.1], [0, 0, 0, 1]]), (0.32, 0.34, 0.37))
        rospy.sleep(1)

        # Main function: actual behaviour of the robot
        grasped = self.grasp([[-0.15, 0.6, 0.32], [0, 1, 0, 0]])

        if grasped:
            self.place([[0.411, -0.028, 0.208], [0.707, 0.707, 0, 0]])

        rospy.sleep(5)
        self.camera.set_cognex_strobe(False)


if __name__ == '__main__':
    ManipulateNode().run()
