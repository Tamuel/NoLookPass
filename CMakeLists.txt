cmake_minimum_required(VERSION 2.8.3)
project(no_look_pass_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  message_generation
  actionlib
  actionlib_msgs
  message_filters
)

add_message_files(
  FILES
  move.msg
  vec2d.msg
  laserScan.msg
  master.msg
  estimation.msg
  depth.msg
  bounding.msg
)

add_service_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES no_look_pass_robot
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS})

## Declare a cpp executabl
add_executable(actuation_node src/actuationNode.cpp)
add_executable(perception_node src/perceptionNode.cpp)
add_executable(cognition_node src/cognitionNode.cpp)
add_executable(masterRouteEstimation_node src/masterRouteEstimationNode.cpp)
add_executable(masterPosition_node src/masterPositionNode.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(actuation_node no_look_pass_robot_generate_messages_cpp)
add_dependencies(perception_node no_look_pass_robot_generate_messages_cpp)
add_dependencies(cognition_node no_look_pass_robot_generate_messages_cpp)
add_dependencies(masterRouteEstimation_node no_look_pass_robot_generate_messages_cpp)
add_dependencies(masterPosition_node no_look_pass_robot_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(actuation_node ${catkin_LIBRARIES})
target_link_libraries(perception_node ${catkin_LIBRARIES})
target_link_libraries(cognition_node ${catkin_LIBRARIES})
target_link_libraries(masterRouteEstimation_node ${catkin_LIBRARIES})
target_link_libraries(masterPosition_node ${catkin_LIBRARIES})
