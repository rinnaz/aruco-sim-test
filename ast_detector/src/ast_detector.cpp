#include "ast_detector/ast_detector.h"

MarkerDetector::MarkerDetector()
: m_cameraParamsFile { "/config/head_camera.yaml" },
  m_detectorParamsFile { "/config/detector_parameters.yaml" },
  m_package_path { ros::package::getPath("ast_detector") },
  m_cameraTopicName { "/ast_source_cam/image_raw" },
  m_imageTopicName { "/ast_source_cam/image_with_markers" },
  m_markersTopicName { "/detected_markers" },
  m_dict { cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100) }
{
    //Topics to publish
    m_pub_pose = m_nh.advertise<ast_msgs::Markers>(m_markersTopicName, 1);
    m_pub_image = m_nh.advertise<sensor_msgs::Image>(m_imageTopicName, 1);

    //Topic to subscribe
    readDetectorParams(m_package_path + m_detectorParamsFile);
    readCameraParams(m_package_path + m_cameraParamsFile);

    m_sub = m_nh.subscribe(m_cameraTopicName, 1, &MarkerDetector::callback, this);
}

MarkerDetector::~MarkerDetector() {}

void MarkerDetector::readCameraParams(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["camera_matrix"] >> m_cameraMatrix;
    fs["distortion_coefficients"] >> m_distCoeffs;
}

void MarkerDetector::readDetectorParams(const std::string &filename)
{
    m_detectorParams = cv::aruco::DetectorParameters::create();
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["adaptiveThreshWinSizeMin"] >> m_detectorParams->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> m_detectorParams->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> m_detectorParams->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> m_detectorParams->adaptiveThreshConstant;    
    fs["minMarkerPerimeterRate"] >> m_detectorParams->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> m_detectorParams->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> m_detectorParams->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> m_detectorParams->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> m_detectorParams->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> m_detectorParams->minMarkerDistanceRate;    
    fs["cornerRefinementWinSize"] >> m_detectorParams->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> m_detectorParams->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> m_detectorParams->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> m_detectorParams->markerBorderBits;

    fs["perspectiveRemovePixelPerCell"] >> m_detectorParams->perspectiveRemovePixelPerCell;    
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> m_detectorParams->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> m_detectorParams->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> m_detectorParams->minOtsuStdDev;
    fs["errorCorrectionRate"] >> m_detectorParams->errorCorrectionRate;

    m_detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

void MarkerDetector::callback(const sensor_msgs::Image::ConstPtr &img) const
{
    ast_msgs::Markers markers_msg;
    ast_msgs::MarkerPose pose_msg;

    sensor_msgs::Image image_msg;

    auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

    cv::Mat rotationMatrix;

    cv::Mat imageCopy;
    (cv_ptr->image).copyTo(imageCopy);
  
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::aruco::detectMarkers(imageCopy, 
                              m_dict, 
                              markerCorners, 
                              markerIds, 
                              m_detectorParams);

    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, 
                                         m_cameraMatrix, m_distCoeffs, 
                                         rvecs, tvecs);

    // draw axis for each marker
    for (auto i { 0 }; i < markerIds.size(); i++)
    {
        cv::aruco::drawAxis(imageCopy, m_cameraMatrix, m_distCoeffs, 
                            rvecs[i], tvecs[i], 0.05);
    }
    

    // cv::imshow("Marker_detector", imageCopy);
    // cv::waitKey(1);
    
    for (auto i { 0 }; i < markerIds.size(); i++)
    {
        cv::Rodrigues(rvecs[i], rotationMatrix);

        markers_msg.markerIds_m.push_back(markerIds[i]);
        
        for (auto j : { 0, 1, 2 })
        {
            pose_msg.tvecs_m.push_back(tvecs[i][j]);
            pose_msg.z_rot_m.push_back(rotationMatrix.at<double>(j, 2));
        }

        markers_msg.poses_m.push_back(pose_msg);
        pose_msg.tvecs_m.clear();
        pose_msg.z_rot_m.clear();
    }
    
    m_pub_pose.publish(markers_msg);
    
    cv_ptr->image = imageCopy;
    (*cv_ptr).toImageMsg(image_msg);
    m_pub_image.publish(image_msg);
} 

