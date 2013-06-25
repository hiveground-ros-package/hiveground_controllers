cmake_minimum_required(VERSION 2.8.3)
project(ptp_controller)

find_package(catkin REQUIRED COMPONENTS roscpp)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS boost
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/ptp_controller.cpp           
)

## Declare a cpp executable
#add_executable(${PROJECT_NAME}
#)

#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)


