#include "ast_detector/ast_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_detector");

  ROS_WARN("ast_detector process started");

  MarkerDetector detector;

  ros::spin();
  return 0;
}
 
