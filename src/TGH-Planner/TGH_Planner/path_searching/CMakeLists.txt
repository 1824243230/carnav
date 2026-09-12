cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  nav_msgs
  
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS plan_env 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG_MODE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -Wall")
endif()

add_library( path_searching 
    src/kinodynamic_astar.cpp 
    src/kinodynamic_astar_2D.cpp 
    src/astar.cpp
    src/astar_2D.cpp
    src/risk_aware_edge.cpp
    src/risk_aware_path_selector.cpp
    src/path_reliability/PathReliabilityEvaluator.cpp
    src/topo_prm.cpp
    src/dubins.cpp
    src/jump_point_search.cpp
    )
target_link_libraries( path_searching
    ${catkin_LIBRARIES} 
    )  

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(path_reliability_evaluator_test
    test/path_reliability/PathReliabilityEvaluatorTest.cpp
    test/path_reliability/PathReliabilityRankingTest.cpp)
  if(TARGET path_reliability_evaluator_test)
    target_link_libraries(path_reliability_evaluator_test
      path_searching
      ${catkin_LIBRARIES})
  endif()
endif()
