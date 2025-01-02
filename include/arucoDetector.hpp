#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.hpp>

class ArucoDetector
{
  cv_bridge::CvImagePtr mCvPtr_;

public:
  ArucoDetector();
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
  cv::Ptr<cv::aruco::Dictionary> dict_;
  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_;
  cv::Mat currentFrame_;
  void detect(const sensor_msgs::msg::Image::ConstSharedPtr&);
  const sensor_msgs::msg::Image::ConstSharedPtr getFrameAsImgMsg();
};
