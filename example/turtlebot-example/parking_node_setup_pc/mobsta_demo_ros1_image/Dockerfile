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

# we're going to build at the MOBSTA level, 
#   as we need to bring that code in, so this is the path
#   to the directory containing this dockerfile relative to
#   the MOBSTA ROOOT
ENV DOCKER_IMAGE_PATH "example/turtlebot-example/parking_node_setup_pc/mobsta_demo_ros1_image"

RUN if [ -z "$(getent group  ${CUSTOM_GID})" ] ; then addgroup --gid ${CUSTOM_GID} ${CUSTOM_USERNAME}; fi
RUN if [ -z "$(getent passwd ${CUSTOM_UID})" ] ; then adduser --uid ${CUSTOM_UID} --gid ${CUSTOM_GID} --disabled-password --gecos '' ${CUSTOM_USERNAME}; fi
RUN echo "$CUSTOM_USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV SRC_DIR ${SRC_DIR}
RUN mkdir -p ${SRC_DIR}
RUN mkdir -p ${SRC_DIR}/logs

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG DEBIAN_FRONTEND="noninteractive"

RUN apt-get update -y --allow-unauthenticated && \
    apt-get dist-upgrade -y --allow-unauthenticated

RUN sudo apt-get install -y curl build-essential git

RUN chown -R ${CUSTOM_USERNAME}:${CUSTOM_USERNAME} /home/${CUSTOM_USERNAME}
ENV USER ${CUSTOM_USERNAME}
USER ${CUSTOM_USERNAME}
RUN rosdep update
USER root

RUN sudo apt-get update -y


# Install ROS packages
RUN sudo apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
    
# Install turtlebot packages
RUN sudo apt-get install -y ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3

# A few other useful things
RUN apt-get update \
 && apt-get install -y --fix-missing --no-install-recommends \
 tmux \
 python3-setuptools \
 python3-pip \
 software-properties-common \
 ssh \
 vim

# Get newer better cmake, needed for gorgon compilation
RUN mkdir -p ${SRC_DIR}/miniconda3
RUN curl -s https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh > ${SRC_DIR}/miniconda3/miniconda.sh
RUN bash ${SRC_DIR}/miniconda3/miniconda.sh -b -u -p ${SRC_DIR}/miniconda3
RUN rm ${SRC_DIR}/miniconda3/miniconda.sh
RUN ${SRC_DIR}/miniconda3/bin/conda create -n CMake cmake


# Update the automatic parking code
COPY "$DOCKER_IMAGE_PATH"/automatic_parking.py /opt/ros/noetic/lib/turtlebot3_automatic_parking/automatic_parking
COPY "$DOCKER_IMAGE_PATH"/turtlebot3_automatic_parking.rviz /opt/ros/noetic/share/turtlebot3_automatic_parking/rviz/turtlebot3_automatic_parking.rviz
COPY "$DOCKER_IMAGE_PATH"/record-log.py ${SRC_DIR}/record-log.py

# rebuild the demo cause it doesn't exist on noetic
COPY "$DOCKER_IMAGE_PATH"/catkin_ws ${SRC_DIR}/catkin_ws
WORKDIR ${SRC_DIR}/catkin_ws/src
RUN . /opt/ros/noetic/setup.sh && catkin_init_workspace
WORKDIR ${SRC_DIR}/catkin_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make

# build mobsta
COPY --chown=${CUSTOM_USERNAME} . ${SRC_DIR}/MOBSTA

# fails with permissions error
WORKDIR ${SRC_DIR}/MOBSTA
RUN . /opt/ros/noetic/setup.sh && ${SRC_DIR}/miniconda3/bin/conda run --no-capture-output -n CMake ./deps/ros_install_deps.sh

USER ${CUSTOM_USERNAME}
WORKDIR ${SRC_DIR}

# Set turtlebot model name
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=\`hostname -I | awk '{print \$1}'\`" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://\$ROS_HOSTNAME:11311" >> ~/.bashrc


