cmake_minimum_required(VERSION 3.0.2)
project(ast_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_msgs
  rnrt_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rm_planner
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
