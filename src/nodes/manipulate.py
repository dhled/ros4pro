#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from moveit_commander import roscpp_initialize
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
from ros4pro.simulation.gripper import Gripper
from ros4pro.simulation.camera import Camera
from ros4pro.srv import VisionPredict, VisionPredictRequest
from ros4pro.transformations import multiply_transform, list_to_pose2, pose_to_list2, list_to_pose_stamped2
from sensor_msgs.msg import Image
from tf import TransformBroadcaster, TransformListener

class ManipulateNode(object):
    MIN_FRACTION = 0.8     # Minimum percentage of points successfully computed to initiate a cartesian trajectory (%)
    JOINT_JUMP = 5         # Authorized sum of joint angle differences between 2 cartesian points (rad)
    CART_RESOLUTION = .001 # Resolution between 2 cartesian points (m)
    CUBE_HEIGHT = 0.05     # Height of a cube to be grasped
    FEEDER_HEIGHT = 0.38     # Assumption of the height of the feeder
    FEEDER_DEEP = 0.26
    FEEDER_LONG = 0.56
    PALETTE_HEIGHT = 0.13    # Assumption of the height of the palette hosting the robot (under /base in -z)
    PALETTE_WIDTH = 0.79
    FRONT_ROBOT_X = 0.225

    def __init__(self):      
        self.tfb = TransformBroadcaster()
        self.tfl = TransformListener()
        self.commander = MoveGroupCommander("right_arm")
        self.camera = Camera()
        self.gripper = Gripper()
        self.scene = PlanningSceneInterface()
        self.image_camera = None
        rospy.Subscriber("/io/internal_camera/right_hand_camera/image_rect", Image, self._cb_image)

    def _cb_image(self, msg):
        self.image_camera = msg

    def scan(self, enable_vision):
        z = self.FEEDER_HEIGHT - self.CUBE_HEIGHT
        if enable_vision:  
            # Go to the scan position in joint space and wait 4 seconds for the arm to be steady
            scan_joints = [-1.278, -2.023, 2.806, 1.5, 0.114, -0.384, 3.0]

            self.commander.set_joint_value_target(scan_joints)
            success = self.commander.go()
            rospy.sleep(4)

            # Briefly enable light flashing and send image to the vision server to see if there's some cube in there
            self.camera.shoot()
            predict = rospy.ServiceProxy('ros4pro/vision/predict', VisionPredict)
            response = predict.call(VisionPredictRequest(image=self.image_camera))
                
            # For each found cube, compute and return its picking pose as well as its bin label 1 or 2 
            cubes = []
            for i, label_msg in enumerate(response.label):
                label = label_msg.data
                # Scale CUBE(x, y) from pixels to meters wrt right_hand_camera frame
                x = (response.x_center[i].data - 752/2)*0.310/752   
                y = (response.y_center[i].data - 480/2)*0.195/480
                rospy.loginfo("Found cube {} with label {} at position {} wrt right_hand_camera".format(i, label, (x, y, z)))

                camera_T_cube = [[x, y, z], [0, 0, 0, 1]]
                self.tfb.sendTransform(camera_T_cube[0], camera_T_cube[1], rospy.Time.now(), "cube{}".format(i), "right_hand_camera")
                cube_T_gripper = [[0, 0, -z], [0, 0, -1, 0]]
                base_T_camera = self.tfl.lookupTransform("base", "right_hand_camera", rospy.Time(0))
                base_T_cube = multiply_transform(base_T_camera, camera_T_cube)
                self.tfb.sendTransform(base_T_cube[0], base_T_cube[1], rospy.Time.now(), "here","base")
                cubes.append((base_T_cube, label))
            return cubes
        else:
            # This is the hardcoded cube that is assumed to always be at the same location
            return [([[0.13, 0.5, z], [0, 1, 0, 0]], 1)]
    
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
        self.gripper.open()
        if not success:
            rospy.logerr("Can't find a valid path to place pose")
            return False

        return True

    def run(self):
        # Main function: actual behaviour of the robot
        rospy.sleep(1)
        self.scene.add_box("ground", list_to_pose_stamped2([[0, 0, 0], [0, 0, 0, 1]]), (0.65, 0.80, 0.01))
        self.scene.add_box("feeder", list_to_pose_stamped2([[self.FEEDER_LONG/2 - self.FRONT_ROBOT_X, self.FEEDER_DEEP/2 + self.PALETTE_WIDTH/2, self.FEEDER_HEIGHT/2 - self.PALETTE_HEIGHT], [0, 0, 0, 1]]),
                                                            (self.FEEDER_LONG, self.FEEDER_DEEP, self.FEEDER_HEIGHT))
        rospy.sleep(1)

        while not rospy.is_shutdown():
            rospy.loginfo("Scanning the feeder area...")
            cubes = self.scan(enable_vision=False)
            
            for cube, label in cubes:
                if not rospy.is_shutdown():
                    rospy.loginfo("Grasping the found cube...")
                    grasped = self.grasp(cube)

                    if grasped:
                        rospy.loginfo("Grasp is a success! Placing the cube in bin {}".format(label))
                        self.place([[0.411, -0.028, 0.11], [0.707, 0.707, 0, 0]])
            rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node("manipulate_sawyer")
    ManipulateNode().run()
