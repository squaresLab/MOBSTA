#!/bin/bash

# @ 2023 Carnegie Mellon University. All rights reserved.
# National Robotics Engineering Center, Carnegie Mellon University
# www.nrec.ri.cmu.edu
# Confidential and Proprietary - Do not distribute without prior written
# permission.

# License Status: Not Released.
# (License Status to be confirmed by CTTEC prior to release from NREC)
# This notice must appear in all copies of this file and its derivatives.

# NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project
# Manager/PI prior to use on another project).
# Created for Program: HPSTA - 55435.1.1990813

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_HOSTNAME=`hostname -I | awk '{print $1}'`
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger

roslaunch turtlebot3_bringup turtlebot3_robot.launch 
