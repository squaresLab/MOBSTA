#! /bin/bash

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
HOST_MOBSTA_ROOT=$SCRIPTPATH/../../..
DOCKER_MOBSTA_ROOT=$HOME/MOBSTA
IMAGE_NAME=mobsta_demo_ros1_image

# # Use this command for debug/development
# SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
# HOST_MOBSTA_ROOT=$SCRIPTPATH/../../..
# DOCKER_MOBSTA_ROOT=$HOME/MOBSTA
# docker run -it -v "$HOST_MOBSTA_ROOT:$DOCKER_MOBSTA_ROOT:rw" $IMAGE_NAME /bin/bash
docker run -it $IMAGE_NAME /bin/bash
