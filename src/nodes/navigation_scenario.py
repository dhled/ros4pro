#! /usr/bin/python
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


if __name__ == "__main__":
    try:
        rospy.init_node("catie_test_scenario")
        rospy.loginfo("ParticleFilterHandler Initialization")
        particle_handler = particle_filter_handler.ParticleFilterHandler()
        rospy.loginfo("SimpleNavigationGoals Initialization")
        nav_goals = simple_navigation_goals.SimpleNavigationGoals()
        rospy.loginfo("Initializations done")

        # What to do if shut down (e.g. ctrl + C or failure)
        rospy.on_shutdown(nav_goals._shutdown)

        # Setting the assumed initial position of the robot
        # particle_handler.reset_particle_cloud_simple(
        #     0, 0, 0 * math.pi / 4, x_var=0.5, y_var=0.5, theta_var=math.pi / 6
        # )
        rospy.loginfo("Particles resetted")

        while True:
            if not (nav_goals.go_to(0., 0., 0)):
                break
            if not (nav_goals.go_to(-1.5, -0.5, math.pi / 2)):
                break
            if not (nav_goals.go_to(0, 0, 0)):
                break

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())

    rospy.loginfo("test terminated.")
