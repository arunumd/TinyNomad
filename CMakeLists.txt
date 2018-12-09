cmake_minimum_required(VERSION 2.8.3)
project(tinynomad)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_runtime
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   move_base_msgs   sensor_msgs   
 )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
 target_link_libraries(tinynomad
   ${catkin_LIBRARIES}
 )

