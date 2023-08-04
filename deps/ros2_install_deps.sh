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

    # install Gorgon
    # adjustment LOCAL_JSON_LIB can be removed if json is not install locally
    pushd deps/gorgon-mutations-lib/
    cmake -DGORGON_USE_OPENCV=OFF -DLOCAL_JSON_LIB=/usr/include/nlohmann/ -B build
    cmake --build build
    sudo make -C build install
    pushd

    sudo ldconfig

    # install ros_comm rosbag
    pushd deps/rosbag2
    colcon build
    pushd
)