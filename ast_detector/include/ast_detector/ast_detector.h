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
    void readCameraParams(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
    void readDetectorParams(const std::string &filename, cv::Ptr<cv::aruco::DetectorParameters> &detectorParams);
    void callback(const sensor_msgs::Image::ConstPtr &img);

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::string package_path;
    std::string cameraParamsFile;
    std::string detectorParamsFile;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> dict;

    cv_bridge::CvImagePtr cv_ptr;
};