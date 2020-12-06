
# ROS workspace
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

#export ROS_HOSTNAME=$(hostname).local
export ROS_IP=`ip address|grep inet|grep dynamic|tr ' ' ':'|cut -d':' -f6|cut -d'/' -f1|head -n1`
export TURTLEBOT3_MODEL=burger

# CHOOSE A ROS MASTER (add # in front of the lines you want to disable)
ROS_MASTER_URI=http://localhost:11311
#ROS_MASTER_URI=http://poppy.local:11311
#ROS_MASTER_URI=http://raspberrypi.local:11311
