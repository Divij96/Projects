#  ==========================================
#   Title:  receiver node
#   Author: Divij Grover
#   Date:   20 Feb 2022
#  ==========================================




cmake_minimum_required(VERSION 3.0.2)
project(beacon)

add_compile_options(-std=c++11)

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
  transmitters
)

add_message_files(
  FILES
  receiver.msg
  beacon_msg.msg
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

add_executable(receiver_beacon src/receiverNode.cpp)
add_dependencies(receiver_beacon ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
 target_link_libraries(receiver_beacon ${catkin_LIBRARIES})
