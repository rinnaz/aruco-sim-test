#include "ast_detector/ast_detector.h"

MarkerDetector::MarkerDetector()
: cameraParamsFile { "/config/head_camera.yaml" },
  detectorParamsFile { "/config/detector_parameters.yaml" },
  package_path { ros::package::getPath("ast_detector") },
  cameraTopicName { "/ast_source_cam/image_raw" },
  dict { cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100) }
{
    //Topic to publish
    this->pub = nh.advertise<ast_msgs::Markers>("/detected_markers", 1);

    //Topic to subscribe
    this->readDetectorParams(this->package_path + this->detectorParamsFile);
    this->readCameraParams(this->package_path + this->cameraParamsFile);

    this->sub = nh.subscribe(this->cameraTopicName, 
                             1, &MarkerDetector::callback, this);
}

MarkerDetector::~MarkerDetector() {}

void MarkerDetector::readCameraParams(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
}

void MarkerDetector::readDetectorParams(const std::string &filename)
{
    this->detectorParams = cv::aruco::DetectorParameters::create();
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["adaptiveThreshWinSizeMin"] >> detectorParams->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> detectorParams->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> detectorParams->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> detectorParams->adaptiveThreshConstant;    
    fs["minMarkerPerimeterRate"] >> detectorParams->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> detectorParams->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> detectorParams->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> detectorParams->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> detectorParams->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> detectorParams->minMarkerDistanceRate;    
    fs["cornerRefinementWinSize"] >> detectorParams->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> detectorParams->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> detectorParams->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> detectorParams->markerBorderBits;

    fs["perspectiveRemovePixelPerCell"] >> detectorParams->perspectiveRemovePixelPerCell;    
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> detectorParams->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> detectorParams->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> detectorParams->minOtsuStdDev;
    fs["errorCorrectionRate"] >> detectorParams->errorCorrectionRate;

    this->detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

void MarkerDetector::callback(const sensor_msgs::Image::ConstPtr &img) const
{
    ast_msgs::Markers markers_pub;
    ast_msgs::MarkerPose m_pose;

    auto cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

    cv::Mat rotationMatrix;

    cv::Mat imageCopy;
    (cv_ptr->image).copyTo(imageCopy);
  
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    cv::aruco::detectMarkers(imageCopy, 
                              this->dict, 
                              markerCorners, 
                              markerIds, 
                              this->detectorParams);

    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    std::vector<cv::Vec3d> rvecs, tvecs;

    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, 
                                         this->cameraMatrix, this->distCoeffs, 
                                         rvecs, tvecs);

    // draw axis for each marker
    for (auto i { 0 }; i < markerIds.size(); i++)
    {
        cv::aruco::drawAxis(imageCopy, this->cameraMatrix, this->distCoeffs, 
                            rvecs[i], tvecs[i], 0.05);
    }
    
    cv::imshow("Marker_detector", imageCopy);
    cv::waitKey(1);
    
    for (auto i { 0 }; i < markerIds.size(); i++)
    {
        cv::Rodrigues(rvecs[i], rotationMatrix);

        markers_pub.markerIds_m.push_back(markerIds[i]);
        for (auto j : { 0, 1, 2 })
        {
            m_pose.tvecs_m.push_back(tvecs[i][j]);
            m_pose.z_rot_m.push_back(rotationMatrix.at<double>(j, 2));
        }

        markers_pub.poses_m.push_back(m_pose);
        m_pose.tvecs_m.clear();
        m_pose.z_rot_m.clear();
    }

    this->pub.publish(markers_pub);
} 

