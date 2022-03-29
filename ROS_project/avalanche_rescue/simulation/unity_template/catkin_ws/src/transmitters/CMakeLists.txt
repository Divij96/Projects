cmake_minimum_required(VERSION 3.0.2)
project(transmitters)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
message_generation
roscpp
std_msgs
roslib
tf
tf2_msgs
mav_msgs
eigen_conversions
tf_conversions
rospy
)

add_message_files(
  FILES
  transmitter.msg
  transmitter_array.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  mav_msgs
  tf
  eigen_conversions
  tf_conversions
)

find_package(Eigen3)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

add_executable(transmitter_beacon src/transmitterNode.cpp)
add_dependencies(transmitter_beacon ${catkin_EXPORTED_TARGETS})

add_dependencies(transmitter_beacon transmitters_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(transmitter_beacon ${catkin_LIBRARIES})
