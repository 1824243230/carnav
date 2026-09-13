cmake_minimum_required(VERSION 2.8.3)
project(save_map_point)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  pcl_ros
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (save_map_point src/save_map_point.cpp )
target_link_libraries(save_map_point
  ${catkin_LIBRARIES}
  )

add_executable (egvg_vis src/egvg_vis.cpp )
target_link_libraries(egvg_vis
  ${catkin_LIBRARIES}
  )

add_executable (pointcloud_accumulator src/pointcloud_accumulator.cpp )
target_link_libraries(pointcloud_accumulator
  ${catkin_LIBRARIES}
  )

# Keep the utility package usable from both devel/setup.bash and an installed
# workspace.  Without these rules, catkin_make install omits its nodes and
# launch files, causing roslaunch to report that map_save.launch is missing.
install(TARGETS
  save_map_point
  egvg_vis
  pointcloud_accumulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/egvg_vis.py
  src/pub_map.py
  src/pub_map_img.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch map_point
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
