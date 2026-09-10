#!/usr/bin/env bash
# Build with the Ubuntu 20.04 libraries used by ROS Noetic.
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/noetic/setup.bash
set -o pipefail
mkdir -p logs

# Clear PCL's component cache as well as selecting its package configuration:
# otherwise CMake can retain libraries from a previous /usr/local PCL build.
catkin_make -j4 -l4 \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DEigen3_DIR=/usr/lib/cmake/eigen3 \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 \
  --cmake-args -U 'PCL_*' \
  -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl \
  2>&1 | tee logs/build.log
