BASE_IMAGE=arm64v8/ros:kinetic
pushd ../parking_node_setup_pc
./build_ros1_image.sh $BASE_IMAGE
popd
