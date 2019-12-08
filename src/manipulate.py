#!/usr/bin/env python

import rospy
from moveit_commander.move_group import MoveGroupCommander
from intera_interface.gripper import Gripper

class ManipulateNode(object):
    def __init__(self):
        rospy.init_node("manipulate_sawyer")
        self.gripper = Gripper()
        self.commander = MoveGroupCommander("right_arm")

    def run(self):
        # Main function: actual behaviour of the robot
        self.commander.set_pose_target([0.526, 0.477, 0.304, 0.702, -0.095, 0.705, 0.034])
        self.go()
        self.gripper.close()
        rospy.sleep(5)
        self.gripper.open()

if __name__ == '__main__':
    ManipulateNode().run()
