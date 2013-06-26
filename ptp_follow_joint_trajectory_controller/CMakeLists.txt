cmake_minimum_required(VERSION 2.8.3)
project(ptp_follow_joint_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS roscpp controller_interface realtime_tools actionlib)

# Declare catkin package
catkin_package(  
  INCLUDE_DIRS include
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/ptp_follow_joint_trajectory_controller.cpp
)

target_link_libraries(
  ${PROJECT_NAME}  
  ${catkin_LIBRARIES}
)


