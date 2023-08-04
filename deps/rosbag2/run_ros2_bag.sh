#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

source ${SCRIPT_DIR}/install/local_setup.bash
# ros2 bag mutation_play $1 --mutation-config-file $2
# ros2 bag mutation_play rosbag2_2023_07_14-16_01_44/ --mutation-config-file mutation_config.json
ros2 bag mutation_play $1 --mutation-config-file $2
RETURN_CODE=$?

# if ros2 bag exits badly for any reason, it messes up all the terminal settings. Rescue them
if [ $RETURN_CODE -ne 0 ]
then
  stty sane
  exit $RETURN_CODE
fi