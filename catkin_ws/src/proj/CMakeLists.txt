
cmake_minimum_required(VERSION 2.8.3)
project(proj)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)
find_package(PCL 1.2 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (proj_node src/proj_node.cpp)
target_link_libraries (proj_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
