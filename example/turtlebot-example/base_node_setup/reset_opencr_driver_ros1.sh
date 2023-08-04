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

docker run -it --privileged --log-driver=syslog --net=host \
	--group-add `getent group dialout | cut -d: -f3` \
	--env="COLUMNS" --env="LINES" --env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "/etc/localtime:/etc/localtime:ro" turtlebot_ros1_image \
	bash -c "cd $HOME/opencr_update && ./update.sh /dev/ttyACM0 burger.opencr"
