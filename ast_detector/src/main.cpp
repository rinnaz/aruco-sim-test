#include "ast_detector/ast_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");

  ROS_WARN("ast_detector process started");

  MarkerDetector detector;

  ros::spin();
  return 0;
}
 
