cmake_minimum_required(VERSION 2.8.3)
project(tinynomad)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_generation
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_runtime
  cv_bridge
)

catkin_package()
###########
## Build ##
###########

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui opencv_imgcodecs)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(tinynomad src/Navigation.cpp src/main.cpp src/Turtlebot.cpp src/Vision.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tinynomad ${catkin_LIBRARIES} ${OpenCV_LIBS})

###########
# Testing #
###########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(tinynomadTest test/turtleTest.launch test/tinynomadTest.cpp test/NavigationTest.cpp 
                    test/VisionTest.cpp test/TurtlebotTest.cpp src/Navigation.cpp  
                    src/Vision.cpp src/Turtlebot.cpp)
  target_link_libraries(tinynomadTest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  add_dependencies(tinynomadTest tinynomad ${catkin_EXPORTED_TARGETS})
  #set_target_properties(tinynomadTest PROPERTIES LINKER_LANGUAGE CXX)
endif()

