#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseFeedback

drop_point_1 = PoseStamped()
drop_point_1.header.frame_id = "map"
drop_point_1.pose.position.x = 0.48
drop_point_1.pose.position.y = -1.57
drop_point_1.pose.position.z = 0
drop_point_1.pose.orientation.x = 0.0
drop_point_1.pose.orientation.y = 0.0
drop_point_1.pose.orientation.z = 0.692725772442
drop_point_1.pose.orientation.w = 0.721201084438

drop_point_2 = PoseStamped()
drop_point_2.header.frame_id = "map"
drop_point_2.pose.position.x = 0.336820632219
drop_point_2.pose.position.y = 1.71840572357    
drop_point_2.pose.position.z = 0
drop_point_2.pose.orientation.x = 0.0
drop_point_2.pose.orientation.y = 0.0
drop_point_2.pose.orientation.z = -0.659250448921
drop_point_2.pose.orientation.w = 0.751923430675

sawyer_point = PoseStamped()
sawyer_point.header.frame_id = "map"
sawyer_point.pose.position.x = -1.99812340736
sawyer_point.pose.position.y = 1.07787430286
sawyer_point.pose.position.z = 0
sawyer_point.pose.orientation.x = 0.0
sawyer_point.pose.orientation.y = 0.0
sawyer_point.pose.orientation.z = 0
sawyer_point.pose.orientation.w = 1


import actionlib
from move_base_msgs.msg import (
    MoveBaseActionGoal,
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseActionFeedback,
)
from actionlib_msgs.msg import GoalStatus


class InitException(Exception):
    pass


# Useful doc :
# http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html


class SimpleNavigationGoals:
    """
    Interface with the move_base actionlb. Allows to set a position goal (x, y, theta) that will be followed by a robot, using a predefined GlobalPlanner and LocalPlanner

    Raises:
        InitException --
    """
    
    def __init__(self):
        rospy.loginfo("Starting simple_navigation_goals...")

        # Created the client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up...")
        success = self.move_base.wait_for_server(rospy.Duration(5))
        if not success:
            rospy.logerr("The move_base action server did not respond in time, exiting.")
            raise InitException("Failed SimpleNavigationGoals init")


    def send_goal(self, pose_stp):
        """Sets a goal for the move_base to follow.

        Arguments:
            pose_stp: pose of the goal

        Returns:
            Nothing
        """
        self.goal = MoveBaseGoal()
        self.goal.target_pose = pose_stp 
        self.move_base.send_goal(self.goal)

    def is_arrived(self):
        """
        First return value is True if robot arrived and False if going to goal
        Second return value is True if move base failed to reach the goal and False if goal is proceed
        """
        state = self.move_base.get_state()
        rospy.logwarn("STATUS :  {}".format(state))
        if (state == GoalStatus.SUCCEEDED):
            return True, False
        if (state == GoalStatus.PENDING or state == GoalStatus.ACTIVE) :
            return False, False
        else:
            return False, True
            
    # uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    # uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    # uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
    #                             #   and has since completed its execution (Terminal State)
    # uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    #                             #    to some failure (Terminal State)
    # uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
    #                             #    because the goal was unattainable or invalid (Terminal State)
    # uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
    #                             #    and has not yet completed execution
    # uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
    #                             #    but the action server has not yet confirmed that the goal is canceled
    # uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
    #                             #    and was successfully cancelled (Terminal State)
    # uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
    #                             #    sent over the wire by an action server     

def go_to_point_2():
    return

def go_back_to_sawyer():
    return

def is_arrived():
    return True, False

def send_velocity(value):
    return 


move_base_simple = None


# Annoncing what we're going to do and going to a predefined position
class Waiting_for_cube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["cube_received", "nothing"])
        self.name = "WAITING_FOR_CUBE"
        self.rate = rospy.Rate(1)
        self.cube_param_name = "ros4pro/cube"

    def execute(self, userdata):
        global move_base_simple
        rospy.loginfo_throttle(1,"Executing state {}".format(self.name))
        self.rate.sleep()

        if rospy.has_param(self.cube_param_name):
            rospy.loginfo("NIEC")
            cube_number = rospy.get_param(self.cube_param_name)
            rospy.loginfo("param {}: {}".format(self.cube_param_name, cube_number))
            rospy.delete_param(self.cube_param_name)

            if cube_number == 1:            
                rospy.loginfo("Cube number {} received, going to drop location".format(cube_number))
                move_base_simple.send_goal(drop_point_1)
                return "cube_received"
            elif cube_number == 2:
                move_base_simple.send_goal(drop_point_2)
                rospy.loginfo("Cube number {} received, going to drop location".format(cube_number))
                return "cube_received"
            else:
                return "nothing"
        else:
            # print("param {} not found".format(self.cube_param_name))
            return "nothing"


class Go_to_drop_point(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["arrived", "not_arrived", "failed"])
        self.name = "GO_TO_DROP_POINT"
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        global move_base_simple
        rospy.loginfo_throttle(1,"Executing state {}".format(self.name))
        self.rate.sleep()

        arrived, failed = move_base_simple.is_arrived()

        if failed:
            rospy.logerr("Failed to reach the drop location, going back to the sawyer")
            move_base_simple.send_goal(sawyer_point)
            return "failed"
        elif arrived:
            rospy.loginfo("Arrived at drop location")
            return "arrived"
        else:
           return "not_arrived"


class Drop_cube(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["dropping", "cube_dropped"])
        self.name = "DROP_CUBE"
        self.rate = rospy.Rate(10)
        self.rotation_time = rospy.Duration(3)
        self.start_time = rospy.Time(0)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist)
        self.cmd_vel = Twist()
        self.rotation_speed = 0.7

    def execute(self, userdata):
        global move_base_simple
        rospy.loginfo_throttle(1,"Executing state {}".format(self.name))
        self.rate.sleep()

        if self.start_time == rospy.Time(0):
            self.start_time = rospy.Time.now()
        if rospy.Time.now() - self.start_time > self.rotation_time:
            self.cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel)
            self.start_time = rospy.Time(0)
            rospy.loginfo("Cube successfully dropped, going back to the sawyer")
            move_base_simple.send_goal(sawyer_point)
            return "cube_dropped"
        else:
            self.cmd_vel.angular.z = self.rotation_speed
            self.cmd_vel_pub.publish(self.cmd_vel)
            return "dropping"
    

class Go_back_to_sawyer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["arrived", "not_arrived"])
        self.name = "GO_BACK_TO_SAWYER"
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        global move_base_simple
        rospy.loginfo_throttle(1,"Executing state {}".format(self.name))
        self.rate.sleep()

        arrived, failed = move_base_simple.is_arrived()

        if failed:
            rospy.logerr("Failed to reach the drop location, going back to the sawyer")
            move_base_simple.send_goal(sawyer_point)
            return "failed"
        elif arrived:
            rospy.loginfo("Arrived at drop location")
            return "arrived"
        else:
           return "not_arrived"



def main():
    global move_base_simple

    rospy.init_node("tb3_labyrinthe")
    move_base_simple = SimpleNavigationGoals()
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=["final"])

    # Open the container
    with sm_top:
        smach.StateMachine.add(
            "WAITING_FOR_CUBE",
            Waiting_for_cube(),
            transitions={
                "cube_received": "GO_TO_DROP_POINT",
                "nothing": "WAITING_FOR_CUBE",
            },
        )

        smach.StateMachine.add(
            "GO_TO_DROP_POINT",
            Go_to_drop_point(),
            transitions={
                "arrived": "DROP_CUBE",
                "not_arrived": "GO_TO_DROP_POINT",
                "failed": "GO_BACK_TO_SAWYER",
            },
        )

        smach.StateMachine.add(
            "DROP_CUBE",
            Drop_cube(),
            transitions={
                "dropping": "DROP_CUBE",
                "cube_dropped": "GO_BACK_TO_SAWYER",
            },
        )

        smach.StateMachine.add(
            "GO_BACK_TO_SAWYER",
            Go_back_to_sawyer(),
            transitions={
                "not_arrived": "GO_BACK_TO_SAWYER",
                "arrived": "WAITING_FOR_CUBE",
            },
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("labyrinthe_sm", sm_top, "/SM_ROOT")
    sis.start()
    rospy.loginfo("Starting the state machine")

    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.loginfo("Stoping carry_my_luggage state machine")

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())

    rospy.loginfo("test terminated.")
