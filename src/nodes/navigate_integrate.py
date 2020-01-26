#!/usr/bin/env python
import rospy
import actionlib
import math
from math import radians, degrees

from geometry_msgs.msg import Point
import tf
import traceback
import time
import sys
from simple_navigation_goals import simple_navigation_goals
from catie_particle_handler import particle_filter_handler
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

def main():
    global move_base_simple

    rospy.init_node("tb3_labyrinthe")
    rospy.loginfo("ParticleFilterHandler Initialization")
    particle_handler = particle_filter_handler.ParticleFilterHandler()
    rospy.loginfo("SimpleNavigationGoals Initialization")
    nav_goals = simple_navigation_goals.SimpleNavigationGoals()
    rospy.loginfo("Initializations done")

    # What to do if shut down (e.g. ctrl + C or failure)
    rospy.on_shutdown(nav_goals._shutdown)
    rate_1 = rospy.Rate(1)
    rate_10 = rospy.Rate(10)
    cube_param_name = "ros4pro/cube"

    rotation_time = rospy.Duration(3)
    cmd_vel_pub = rospy.Publisher("/tb3/cmd_vel", Twist, queue_size=10)
    cmd_vel = Twist()
    cmd_vel.angular.z = 0.7

    while not rospy.is_shutdown():

        # Waits for cube param to exist, get the value and then delete the param from the server 
        rospy.loginfo("Waiting for cube number")
        cube_number = -1
        while cube_number < 0:
            if rospy.has_param(cube_param_name):
                cube_number = rospy.get_param(cube_param_name)
                rospy.loginfo("param {}: {}".format(cube_param_name, cube_number))
                rospy.delete_param(cube_param_name)
            else:
                rate_1.sleep()
        
        ret = False
        # Sends nav goal depending on cube number, if it reachs the goal ret is set to True
        if cube_number == 1:            
            rospy.loginfo("Cube number {} received, going to drop location".format(cube_number))

            # DROP POINT 1
            ret = nav_goals.go_to(0, 0.5, math.pi / 2) 

        elif cube_number == 2:
            rospy.loginfo("Cube number {} received, going to drop location".format(cube_number))
            
            # DROP POINT 2
            ret = nav_goals.go_to(-0.5, -0.3, 0) 
        
        else:
            rospy.logwarn("Cube number {} received, number must be 1 and 2, resetting state")
            continue # remaining code of the loop will not be executed
       
        if not ret:
            rospy.logerr("Failed to reach drop point {}, going back to Sawyer".format(cube_number))
            
            # SAWYER POINT
            ret = nav_goals.go_to(1, 0, 0) 
            
            if not ret:
                rospy.logfatal("Failed to reach Sawyer point after failing drop, exiting node")
                return # exit
            else:
                rospy.logwarn("Sawyer reached, resetting state")
                continue
        else:
            rospy.sleep(1)
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rotation_time:
                cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("drop")
                rate_10.sleep()
            rospy.sleep(1)

            cmd_vel_pub.publish(Twist())
            rospy.loginfo("Cube successfully dropped, going back to the sawyer")
            
            # SAWYER POINT
            ret = nav_goals.go_to(1, 0, 0) 
            
            if not ret:
                rospy.logfatal("Failed to reach Sawyer point, exiting node")
                return # exit


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())
