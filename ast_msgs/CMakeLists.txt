cmake_minimum_required(VERSION 3.0.2)
project(ast_msgs)

find_package(catkin REQUIRED COMPONENTS
message_generation  
std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   MarkerPose.msg
   Markers.msg
#   Message1.msg
#   Message2.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
