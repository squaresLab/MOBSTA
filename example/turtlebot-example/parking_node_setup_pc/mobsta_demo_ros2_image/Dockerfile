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
    
RUN sudo apt-get install -y curl build-essential git
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN chown -R ${CUSTOM_USERNAME}:${CUSTOM_USERNAME} /home/${CUSTOM_USERNAME}
ENV USER ${CUSTOM_USERNAME}
USER ${CUSTOM_USERNAME}
RUN rosdep update
USER root

RUN sudo apt-get update -y


# Install ROS packages
RUN sudo apt install -y python3-colcon-common-extensions

RUN sudo apt install -y ros-dashing-cartographer \
  ros-dashing-cartographer-ros \
  ros-dashing-navigation2 \
  ros-dashing-nav2-bringup \ 
  ros-dashing-ros2bag \
  ros-dashing-rosbag2 \
  ros-dashing-rosbag2-storage-default-plugins \
  python3-vcstool \
  python3-pip

# Install turtlebot packages
RUN sudo apt install -y ros-dashing-dynamixel-sdk \
  ros-dashing-turtlebot3-msgs \
  ros-dashing-turtlebot3
  
# Install additional python stuff
RUN sudo pip3 install cython
RUN sudo pip3 install numpy
RUN sudo pip3 install --upgrade pip

# A few other useful things
RUN apt-get update \
 && apt-get install -y --fix-missing --no-install-recommends \
 software-properties-common \
 ssh \
 vim

 
USER ${CUSTOM_USERNAME}

# For the parking demo
RUN mkdir -p ~/turtlebot3_ws/src && \
  cd ~/turtlebot3_ws/src && \
  git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_applications.git && \
  cd ~/turtlebot3_ws/ && \
  /bin/bash -c "source /opt/ros/dashing/setup.bash && colcon build --symlink-install"

WORKDIR ${SRC_DIR}

COPY turtlebot3_automatic_parking.py ${SRC_DIR}/turtlebot3_ws/src/turtlebot3_applications/turtlebot3_automatic_parking/turtlebot3_automatic_parking/turtlebot3_automatic_parking/turtlebot3_automatic_parking.py
COPY turtlebot3_automatic_parking.rviz ${SRC_DIR}/turtlebot3_ws/install/turtlebot3_automatic_parking/share/turtlebot3_automatic_parking/rviz/turtlebot3_automatic_parking.rviz
COPY record-log.py ${SRC_DIR}/record-log.py

# Set turtlebot model name
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
RUN echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=\`hostname -I | awk '{print \$1}'\`" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://\$ROS_HOSTNAME:11311" >> ~/.bashrc
