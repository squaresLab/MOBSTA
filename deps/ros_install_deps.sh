#!/bin/bash
# /****
# * © 2023 Carnegie Mellon University. All rights reserved.
# * National Robotics Engineering Center, Carnegie Mellon University
# * www.nrec.ri.cmu.edu
# * Confidential and Proprietary - Do not distribute without prior written permission.
# *
# * License Status: Not Released.
# * (License Status to be confirmed by CTTEC prior to release from NREC)
# * This notice must appear in all copies of this file and its derivatives.
# ****/

# /****
# * NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project Manager/PI prior to use on another project).
# * Created for Program: HPSTA - 55435.1.1990813
# ****/
(
    set -e
    set -x

    # sudo apt install -y build-essential cmake

    # # install ros 
    # sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    # sudo apt install -y curl # if you haven't already installed curl
    # curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    # sudo apt update
    # sudo apt install -y ros-noetic-ros-base ros-noetic-roscpp

    # install Gorgon
    pushd deps/gorgon-mutations-lib/
    cmake -DCMAKE_INSTALL_PREFIX=./lib -DGORGON_USE_OPENCV=OFF -B build
    cmake --build build
    sudo make -C build install
    pushd

    # install Gorgon-Ros
    pushd deps/gorgon-ros/
    cmake -DCMAKE_INSTALL_PREFIX=./lib -DGORGON_USE_OPENCV=OFF -B build
    cmake --build build
    sudo make -C build install
    pushd

    # install ros_msg_parser
    pushd deps/ros_msg_parser
    cmake -B build
    sudo make -C build install
    pushd

    # install ros_comm rosbag
    pushd deps/ros_comm/tools/rosbag
    cmake -B build
    make -C build
    pushd
)