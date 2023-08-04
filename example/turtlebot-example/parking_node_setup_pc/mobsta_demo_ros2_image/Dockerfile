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

# We're going to build at the MOBSTA level, 
#   as we need to bring that code in, so this is the path
#   to the directory containing this dockerfile relative to
#   the MOBSTA ROOOT
ENV DOCKER_IMAGE_PATH "example/turtlebot-example/parking_node_setup_pc/mobsta_demo_ros2_image"

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

RUN sudo apt install -y ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \ 
  ros-humble-ros2bag \
  ros-humble-rosbag2 \
  ros-humble-test-msgs \
  ros-humble-rosbag2-storage-default-plugins \
  python3-vcstool \
  python3-pip

# Install turtlebot packages
RUN sudo apt install -y ros-humble-dynamixel-sdk \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3
  
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
  /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# To make it humble compatible, we tweak the launch file
ENV LAUNCH_FILE /home/mobstauser/turtlebot3_ws/install/turtlebot3_automatic_parking/share/turtlebot3_automatic_parking/launch/turtlebot3_automatic_parking.launch.py
RUN sed -i "s/node_executable/executable/g" $LAUNCH_FILE
RUN sed -i "s/node_name/name/g" $LAUNCH_FILE

# build mobsta
COPY --chown=${CUSTOM_USERNAME} . ${SRC_DIR}/MOBSTA

# fails with permissions error
WORKDIR ${SRC_DIR}/MOBSTA
RUN . /opt/ros/humble/setup.sh && ./deps/ros2_install_deps.sh

COPY "$DOCKER_IMAGE_PATH"/turtlebot3_automatic_parking.py ${SRC_DIR}/turtlebot3_ws/src/turtlebot3_applications/turtlebot3_automatic_parking/turtlebot3_automatic_parking/turtlebot3_automatic_parking/turtlebot3_automatic_parking.py
COPY "$DOCKER_IMAGE_PATH"/turtlebot3_automatic_parking.rviz ${SRC_DIR}/turtlebot3_ws/install/turtlebot3_automatic_parking/share/turtlebot3_automatic_parking/rviz/turtlebot3_automatic_parking.rviz
COPY "$DOCKER_IMAGE_PATH"/record-log.py ${SRC_DIR}/record-log.py

# Set turtlebot model name
RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=\`hostname -I | awk '{print \$1}'\`" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://\$ROS_HOSTNAME:11311" >> ~/.bashrc
