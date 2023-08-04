BASE_IMAGE=ros:noetic

if [ $# -gt 0 ]; then
    BASE_IMAGE=$1
fi

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
HOST_MOBSTA_ROOT=$SCRIPTPATH/../../..

cd $HOST_MOBSTA_ROOT

DOCKER_ROOT=$HOME
IMAGE_NAME=mobsta_demo_ros1_image

docker build --network=host -t $IMAGE_NAME \
       --build-arg from=`echo $BASE_IMAGE` \
       . \
       --build-arg CUSTOM_UID=$UID \
       --build-arg CUSTOM_USERNAME=$USER \
       --build-arg SRC_DIR=$DOCKER_ROOT \
       -f $SCRIPTPATH/$IMAGE_NAME/Dockerfile
    #    --no-cache \