#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import roscpp_initialize
from moveit_commander.conversions import list_to_pose, list_to_pose_stamped, pose_to_list
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
from intera_interface.gripper import Gripper
from intera_interface.camera import Cameras
from tf import TransformBroadcaster

list_to_pose2 = lambda x: list_to_pose(x[0] + x[1])
pose_to_list2 = lambda x: [pose_to_list(x.pose)[:3], pose_to_list(x.pose)[3:]]
list_to_pose_stamped2 = lambda x: list_to_pose_stamped(x[0] + x[1], "base")

class ManipulateNode(object):
    MIN_FRACTION = 0.8     # Minimum percentage of points successfully computed to initiate a cartesian trajectory (%)
    JOINT_JUMP = 5         # Authorized sum of joint angle differences between 2 cartesian points (rad)
    CART_RESOLUTION = .001 # Resolution between 2 cartesian points (m)

    def __init__(self):
        rospy.init_node("manipulate_sawyer")
        joint_state_topic = ['joint_states:=/robot/joint_states']
        roscpp_initialize(joint_state_topic)
        
        self.tfb = TransformBroadcaster()
        self.gripper = Gripper()
        self.commander = MoveGroupCommander("right_arm")
        self.camera = Cameras()
        self.scene = PlanningSceneInterface()

    def scan(self, scan_length=0.1):
        # We're scanning with the camera. Go to scan start pose
        #scan_pose = [[0.062, 0.544, 0.519], [0.983, 0.184, -0.000, -0.010]]      
        #self.commander.set_pose_target(scan_pose[0] + scan_pose[1])
        scan_joints = [-1.0955751953125, -1.92284765625, 3.042580078125, 1.5862626953125, 0.0332744140625, -0.377076171875, 3.331451171875]
        self.commander.set_joint_value_target(scan_joints)
        success = self.commander.go()

        # Scan along +x axis
        scan_pose = pose_to_list2(self.commander.get_current_pose())
        self.tfb.sendTransform(scan_pose[0], scan_pose[1], rospy.Time.now(), "grasp", "base")

        for i in range(10):
            scan_waypoint_position = (scan_pose[0][0] - scan_length)*i/10., scan_pose[0][1], scan_pose[0][2]
            self.commander.set_position_target(scan_waypoint_position)
            if not self.commander.go():
                rospy.logerr("Cannot compute waypoints of scan")
                break
        
        rospy.loginfo("{}% of success".format(int(fraction*100)))
        self.commander.execute(scan)


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
        grasp, fraction = self.commander.compute_cartesian_path(map(list_to_pose2, [pose_approach, pose_grasp]), self.CART_RESOLUTION, self.JOINT_JUMP)
        
        if fraction < self.MIN_FRACTION:
            rospy.logerr("Can't compute a valid path to grasp")
            return False

        rospy.loginfo("{}% of success".format(int(fraction*100)))
        self.commander.execute(grasp)
        self.gripper.close()

        # Go to retreat
        retreat, fraction = self.commander.compute_cartesian_path(map(list_to_pose2, [pose_grasp, pose_approach]), self.CART_RESOLUTION, self.JOINT_JUMP)

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
        self.scene.add_box("ground", list_to_pose_stamped2([[0, 0, 0], [0, 0, 0, 1]]), (0.65, 0.80, self.CART_RESOLUTION))
        self.scene.add_box("feeder", list_to_pose_stamped2([[-0.2, 0.57, 0.1], [0, 0, 0, 1]]), (0.32, 0.34, 0.37))
        rospy.sleep(1)
        self.scan()
        return
        # Main function: actual behaviour of the robot
        grasped = self.grasp([[-0.15, 0.6, 0.32], [0, 1, 0, 0]])

        if grasped:
            self.place([[0.411, -0.028, 0.208], [0.707, 0.707, 0, 0]])

        rospy.sleep(5)
        self.camera.set_cognex_strobe(False)


if __name__ == '__main__':
    ManipulateNode().run()
