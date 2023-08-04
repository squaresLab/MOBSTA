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


ARG from 
FROM ${from}

ARG CUSTOM_UID
ARG CUSTOM_USERNAME
ARG SRC_DIR

USER root

ENV CUSTOM_UID ${CUSTOM_UID}
ENV CUSTOM_GID ${CUSTOM_UID}
ENV CUSTOM_USERNAME ${CUSTOM_USERNAME}
ENV DEBIAN_FRONTEND=noninteractive

RUN if [ -z "$(getent group  ${CUSTOM_GID})" ] ; then addgroup --gid ${CUSTOM_GID} ${CUSTOM_USERNAME}; fi
RUN if [ -z "$(getent passwd ${CUSTOM_UID})" ] ; then adduser --uid ${CUSTOM_UID} --gid ${CUSTOM_GID} --disabled-password --gecos '' ${CUSTOM_USERNAME}; fi
RUN echo "$CUSTOM_USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV SRC_DIR ${SRC_DIR}
RUN mkdir -p ${SRC_DIR}

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG DEBIAN_FRONTEND="noninteractive"

RUN apt-get update -y --allow-unauthenticated && \
    apt-get dist-upgrade -y --allow-unauthenticated


# Install dependencies
RUN sudo apt-get install -y ros-kinetic-urdf ros-kinetic-interactive-markers ros-kinetic-xacro \
    ros-kinetic-joy ros-kinetic-rosserial-server ros-kinetic-rosserial-client libudev-dev \
    ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-msgs \
    wget


# Copy files into the image
COPY startup_robot.sh ${SRC_DIR}/startup_robot.sh
RUN chmod 777 ${SRC_DIR}/startup_robot.sh

# Build
RUN mkdir -p ${SRC_DIR}/catkin_ws/src && \
  cd ${SRC_DIR}/catkin_ws/src && \
  git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
  git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
  git clone -b develop https://github.com/ROBOTIS-GIT/ld08_driver.git

RUN cd ${SRC_DIR}/ && \
  wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 && \
  tar -xvf opencr_update.tar.bz2 

RUN sudo chmod 777 ${SRC_DIR}/opencr_update/*
RUN /bin/bash -c 'cd ${SRC_DIR}/opencr_update && ./update.sh /dev/ttyACM0 burger.opencr'
RUN /bin/bash -c 'cd ${SRC_DIR}/catkin_ws && source /opt/ros/kinetic/setup.bash && catkin_make'

# Set turtlebot model name
USER ${CUSTOM_USERNAME}
WORKDIR ${SRC_DIR}
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=\`hostname -I | awk '{print \$1}'\`" >> ~/.bashrc
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export LDS_MODEL=LDS-02" >> ~/.bashrc
RUN echo "export OPENCR_PORT=/dev/ttyACM0" >> ~/.bashrc
RUN echo "export OPENCR_MODEL=burger" >> ~/.bashrc
