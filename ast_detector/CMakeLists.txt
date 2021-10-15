cmake_minimum_required(VERSION 3.0.2)
project(ast_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  ast_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_converter
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime ast_msgs
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ast_detector_l src/ast_detector.cpp)
target_link_libraries(ast_detector_l ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ast_detector src/main.cpp)


add_dependencies(${PROJECT_NAME} ast_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(ast_detector
  ast_detector_l
 )



