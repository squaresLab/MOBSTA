#!/bin/bash

HOST_LOG_DIR=$PWD/../logs
DOCKER_LOG_DIR=$HOME/logs
IMAGE_NAME=hpsta_demo_ros2_image

xhost +local:docker

docker run -it --privileged --log-driver=syslog --net=host \
       --cap-add=SYS_PTRACE \
       --env="QT_GRAPHICSSYSTEM=native" --env="XAUTHORITY=/tmp/.docker.xauth" \
       --env="COLUMNS" --env="LINES" --env="DISPLAY=unix:1" \
       --env "NVIDIA_VISIBLE_DEVICES=all" \
       --env "NVIDIA_DRIVER_CAPABILITIES=compute,utility,display" \
       --env="QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       -v "/etc/localtime:/etc/localtime:ro" \
       -v "$HOST_LOG_DIR:$DOCKER_LOG_DIR:rw" $IMAGE_NAME
