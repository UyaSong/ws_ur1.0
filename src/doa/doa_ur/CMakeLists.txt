cmake_minimum_required(VERSION 2.8.3)
project(doa_ur)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  rospy
  visualization_msgs
  tf
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp rospy visualization_msgs tf std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(interactive_tf src/interactive_tf)
target_link_libraries(interactive_tf ${catkin_LIBRARIES})
