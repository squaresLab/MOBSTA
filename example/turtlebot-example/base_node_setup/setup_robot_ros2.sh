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

cp startup_robot_ros2.sh ~/startup_robot_ros2.sh
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update 
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-ros-base

sudo apt install -y python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
sudo apt install -y ros-humble-hls-lfcd-lds-driver
sudo apt install -y ros-humble-turtlebot3-msgs
sudo apt install -y ros-humble-dynamixel-sdk
sudo apt install -y libudev-dev git
mkdir -p ~/turtlebot3_ws/src 

pushd ~/turtlebot3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
popd

cp main.cpp ~/turtlebot3_ws/src/ld08_driver/src/main.cpp

pushd ~/turtlebot3_ws/
colcon build --symlink-install --parallel-workers 1
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/turtlebot3_ws/install/setup.bash
popd

sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo "export ROS_HOSTNAME=\`hostname -I | awk '{print \$1}'\`" >> ~/.bashrc
echo 'export OPENCR_PORT=/dev/ttyACM0' >> ~/.bashrc
echo 'export OPENCR_MODEL=burger' >> ~/.bashrc

export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger

pushd ~/
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf ./opencr_update.tar.bz2

pushd ~/opencr_update
sudo dpkg --add-architecture armhf
sudo apt-get update
sudo apt-get install -y libc6:armhf
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
popd

popd

sudo apt-get install -y openssh-client openssh-server net-tools
sudo service ssh start
sudo systemctl enable ssh
sudo systemctl start ssh

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker ${USER}
sudo chmod 666 /var/run/docker.sock
rm get-docker.sh
