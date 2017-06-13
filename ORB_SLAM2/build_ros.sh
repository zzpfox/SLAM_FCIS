echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
# cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j
