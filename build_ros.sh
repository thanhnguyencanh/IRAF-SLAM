echo "Building ROS nodes"

cd Test/ROS/IRAF_SLAM
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
