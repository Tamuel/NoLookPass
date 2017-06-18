cmake_minimum_required(VERSION 2.8.3)
project(no_look_pass_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_generation
  actionlib
  actionlib_msgs
)

add_message_files(
  FILES
  move.msg
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

include_directories( ${catkin_INCLUDE_DIRS} )

## Declare a cpp executabl
add_executable(actuation_node src/actuationNode.cpp)
add_executable(perception_node src/perceptionNode.cpp)


## Add cmake target dependencies of the executable/library
add_dependencies(actuation_node no_look_pass_robot_generate_messages_cpp)
add_dependencies(perception_node no_look_pass_robot_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(actuation_node ${catkin_LIBRARIES})
target_link_libraries(perception_node ${catkin_LIBRARIES})
