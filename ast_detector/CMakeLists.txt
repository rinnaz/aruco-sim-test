cmake_minimum_required(VERSION 3.0.2)
project(ast_detector)

find_package(catkin REQUIRED COMPONENTS
  rnrt_aruco_detector
)

## System dependencies are found with CMake's conventions

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_converter
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime ast_msgs
#  DEPENDS system_lib
)


include_directories(
#     include
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against



