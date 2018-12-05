cmake_minimum_required(VERSION 2.8.3)
project(tinynomad)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  octomap REQUIRED
)

include_directories(${GTEST_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

target_link_libraries (${OCTOMAP_LIBRARIES})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
 #add_message_files(
   #FILES
   #Num.msg
    #)

## Generate services in the 'srv' folder
 #add_service_files(
   #FILES
   #service.srv
 #)


## Generate added messages and services with any dependencies listed here
 #generate_messages(
   #DEPENDENCIES
   #std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES tinynomad
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  #DEPENDS system_lib
)

#####################################
## Build  talker and listener nodes##
#####################################

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp ${beginner_tutorials_EXPORTED_TARGETS})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  #add_rostest_gtest(
  #testTalker
  #test/test.launch
  #test/test.cpp
  #)
 # target_link_libraries(
  #testTalker
  #${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread
  #)

endif()

