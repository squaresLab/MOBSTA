#!/bin/bash

IMAGE_NAME=mobsta_demo_ros1_image

docker run -it --privileged --log-driver=syslog --net=host $IMAGE_NAME roslaunch turtlebot3_automatic_parking turtlebot3_automatic_parking.launch 
