# ArUco simulation test

Packages for accuracy and precision analysis of the ArUco pose estimation using Gazebo simulator

<img src="images/gazebo.png" width="500">

<img src="images/detected markers.png" width="500">


## How to start

`roslaunch ast_world basic_world.launch`

`rosrun ast_detector ast_detector`

`roslaunch ast_controller main_script.launch`

`roslaunch ast_controller analyze.launch`
