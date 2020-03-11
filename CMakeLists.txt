cmake_minimum_required(VERSION 2.8.3)
project(group2_rwa3)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS osrf_gear moveit_ros_planning_interface geometry_msgs std_msgs roscpp sensor_msgs std_srvs tf2_geometry_msgs trajectory_msgs
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(group2_rwa3 src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(group2_rwa3 ${catkin_LIBRARIES})
