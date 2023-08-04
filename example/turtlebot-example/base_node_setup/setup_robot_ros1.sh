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

sudo apt update
sudo apt install -y curl net-tools
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker ${USER}

IMAGE_NAME=turtlebot_ros1_image
DOCKER_ROOT=$HOME
BASE_IMAGE=arm32v7/ros:kinetic

sudo chmod 666 /var/run/docker.sock
docker build --network=host -t $IMAGE_NAME \
       --build-arg from=`echo $BASE_IMAGE` \
       --no-cache \
       ./$IMAGE_NAME --build-arg CUSTOM_UID=$UID --build-arg CUSTOM_USERNAME=$USER --build-arg SRC_DIR=$DOCKER_ROOT

sudo apt-get install -y openssh-client openssh-server
sudo service ssh start
sudo systemctl enable ssh
sudo systemctl start ssh
cp startup_robot_ros1.sh ~/startup_robot_ros1.sh

rm get-docker.sh
