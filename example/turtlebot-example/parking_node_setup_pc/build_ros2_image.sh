BASE_IMAGE=ros:dashing

if [ $# -gt 0 ]; then
    BASE_IMAGE=$1
fi

DOCKER_ROOT=$HOME
IMAGE_NAME=hpsta_demo_ros2_image

docker build --network=host -t $IMAGE_NAME \
       --build-arg from=`echo $BASE_IMAGE` \
       --no-cache \
       ./$IMAGE_NAME --build-arg CUSTOM_UID=$UID --build-arg CUSTOM_USERNAME=$USER --build-arg SRC_DIR=$DOCKER_ROOT
