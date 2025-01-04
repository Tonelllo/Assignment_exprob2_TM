#include "arucoDetector.hpp"
#include <iostream>
#include <sensor_msgs/msg/detail/image__struct.hpp>

/**
 * @brief Creates the aruco detection node
 *
 * This is done with cv::aruco::DICT_ARUCO_ORIGINAL as the dictionary
 */
ArucoDetector::ArucoDetector() {
  detectorParams_ = cv::aruco::DetectorParameters::create();

  /*determine minimum perimeter for marker contour to be detected.*/
  /**/
  /*This is defined as a rate respect to the maximum dimension of the input
   * image (default 0.03).*/
  detectorParams_->minMarkerPerimeterRate = 0.1;
  dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
}

/**
 * @brief Call this function in order to detect the markers.
 *
 * After the call to this function all the members of the class will be updated
 * in order to be accessed.
 *
 * @param[in] img The image on which to do the detection
 */
void ArucoDetector::detect(const sensor_msgs::msg::Image::ConstSharedPtr &img) {
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    return;
  }
  currentFrame_ = mCvPtr_->image;
  cv::aruco::detectMarkers(currentFrame_, dict_, markerCorners_, markerIds_,
                           detectorParams_);
}

const sensor_msgs::msg::Image::ConstSharedPtr
ArucoDetector::getFrameAsImgMsg() {
  return mCvPtr_->toImageMsg();
}
