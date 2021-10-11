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
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
    const std::string m_package_path;
    const std::string m_cameraParamsFile;
    const std::string m_detectorParamsFile;
    const std::string m_cameraTopicName;

    cv::Ptr<cv::aruco::DetectorParameters> m_detectorParams;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    cv::Ptr<cv::aruco::Dictionary> m_dict;
};