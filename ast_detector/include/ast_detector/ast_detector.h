#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <ros/package.h>

#include <ast_msgs/Markers.h>
#include <ast_msgs/MarkerPose.h>

class MarkerDetector
{
public:
    MarkerDetector();
    ~MarkerDetector();
    void readCameraParams(const std::string &filename);
    void readDetectorParams(const std::string &filename);
    void callback(const sensor_msgs::Image::ConstPtr &img) const;

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    const std::string package_path;
    const std::string cameraParamsFile;
    const std::string detectorParamsFile;
    const std::string cameraTopicName;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> dict;
};