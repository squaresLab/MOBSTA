BASE_IMAGE=arm64v8/ros:dashing
pushd ../parking_node_setup_pc
./build_ros2_image.sh $BASE_IMAGE
popd
