cmake_minimum_required(VERSION 3.0.2)
project(ast_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  ast_msgs
  rrt_cv_gibrid
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
   
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_converter
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs message_runtime ast_msgs
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rrt_cv_gibrid_INCLUDE_DIRS} ##gibrid
)


 add_executable(ast_detector src/main.cpp src/ast_detector.cpp)


 add_dependencies(${PROJECT_NAME} ast_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(ast_detector
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${rrt_cv_gibrid_LIBRARIES} ##gibrid
 )



