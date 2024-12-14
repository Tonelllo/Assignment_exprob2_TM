#include "camMover.hpp"
#include <cstddef>
#include <memory>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/float64_multi_array__struct.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * @brief Class for moving the camera of the robot
 *
 * In This case the movement of the camera is done so that It will go towards the closest aruco marker
 */
CamMover::CamMover() : Node("camMover")
{
  mDetectionPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("/assignment/detected_markers", 1);
  mCurrentSearchingIndex_ = 0;
  mImageSubscriber_.subscribe(this, "/camera/image_raw");
  mJointSubscriber_.subscribe(this, "/joint_states");
  mSyncronizer_ = std::make_shared<mSync_>(mSyncPolicy_(10), mImageSubscriber_, mJointSubscriber_);
  mSyncronizer_->setAgePenalty(0.5);
  mSyncronizer_->registerCallback(std::bind(&CamMover::getCurrentFrame, this, _1, _2));

  mVelocityPublisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/camera_velocity_controller/commands", 10);
  startRotation();
}

/**
 * @brief Function to get the different arucos and save their position
 *
 * @param img Image in which to do the detection of the aruco
 * @param js Current joint state of the camera
 */
void CamMover::getCurrentFrame(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                               const sensor_msgs::msg::JointState::ConstSharedPtr& js)
{
  mArucoDetector_.detect(img);
  cv::Mat& cf = mArucoDetector_.currentFrame_;
  /*cv::aruco::drawDetectedMarkers(cf, mArucoDetector_.markerCorners_, mArucoDetector_.markerIds_);*/
  /*cv::imshow("test", cf);*/
  /*cv::waitKey(10);*/
  mCurrentJointPos_ = js->position[0];
  size_t index = 0;
  for (const auto& id : mArucoDetector_.markerIds_)
  {
    auto tl = mArucoDetector_.markerCorners_[index][0];
    auto br = mArucoDetector_.markerCorners_[index][2];
    auto xCenter = (tl.x + br.x) / 2;
    auto itr = std::find_if(mDetectedIds_.begin(), mDetectedIds_.end(), [&id](auto elem) {
      if (elem.first == id)
      {
        return true;
      }
      return false;
    });
    if (mDetectedIds_.size() < 5 && itr == mDetectedIds_.end())
    {
      if (xCenter < (float)cf.cols / 2 - 10 || xCenter > (float)cf.cols / 2 + 10)
        continue;
      std::cout << "Detected new aruco with id: " << id << std::endl;
      // 0 is the index of camera_joint in the array
      mDetectedIds_.insert({ id, mCurrentJointPos_ });
      if (mDetectedIds_.size() == 5)
      {
        /*std::sort(mDetectedIds_.begin(), mDetectedIds_.end());*/
        std::string outString;
        bool first = true;
        for (const auto& elem : mDetectedIds_)
        {
          if (!first)
          {
            outString += "\t";
          }
          else
          {
            first = false;
          }
          outString += std::to_string(elem.first);
        }
        RCLCPP_INFO(this->get_logger(), "Found all markers. The order is: %s", outString.c_str());
        stopRotation();
        mTarget_ = mDetectedIds_.begin()->second;
        std::cout << "Going to marker with id: " << mDetectedIds_.begin()->first << std::endl;
        mTimer_ = this->create_wall_timer(25ms, std::bind(&CamMover::velCallback, this));
      }
    }
    else if (mDetectedIds_.size() == 5)
    {
      // NOTE that the order is from top left clockwise
      // NOTE that the the corners are given with the ORIGINAL order wich means with the correct orientation
      auto tl = mArucoDetector_.markerCorners_[index][0];
      auto br = mArucoDetector_.markerCorners_[index][2];
      auto xCenter = (tl.x + br.x) / 2;
      auto yCenter = (tl.y + br.y) / 2;
      float radius = cv::norm(tl - br) / 2;

      if (xCenter >= (float)cf.cols / 2 - 60 && xCenter <= (float)cf.cols / 2 + 60)
      {
        auto iter = mDetectedIds_.begin();
        std::advance(iter, mCurrentSearchingIndex_);
        if (id == iter->first)
        {
          RCLCPP_INFO(this->get_logger(), "Published image of marker id: %d", id);
          std::string displayString = "Id: " + std::to_string(id);
          cv::circle(cf, cv::Point(xCenter, yCenter), radius, cv::Scalar(0, 255, 0), 3);
          cv::putText(cf, displayString, cv::Point(xCenter + radius, yCenter - radius), cv::FONT_HERSHEY_COMPLEX, 1,
                      cv::Scalar(255, 0, 0), 3);

          mDetectionPublisher_->publish(*mArucoDetector_.getFrameAsImgMsg());
          ++mCurrentSearchingIndex_;
          if (mCurrentSearchingIndex_ == 5)
            mCurrentSearchingIndex_ = 0;
          auto auxIter = mDetectedIds_.begin();
          std::advance(auxIter, mCurrentSearchingIndex_);
          mTarget_ = auxIter->second;
          std::cout << "Going to marker with id: " << auxIter->first << std::endl;
        }
      }
    }
    ++index;
  }
}

/**
 * @brief Starts the rotation of the camera
 *
 * Note that in the real robot this should be called with a timer, while in simulation only once is enough
 */
void CamMover::startRotation()
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_logger(), "Started rotation");
  std_msgs::msg::Float64MultiArray cmdVel;
  cmdVel.data.emplace_back(0.5);
  while (!mVelocityPublisher_->get_subscription_count())
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for robot to come up");
    rclcpp::sleep_for(1s);
  }
  mVelocityPublisher_->publish(cmdVel);
}

/**
 * @brief Stops the rotation of the robot
 */
void CamMover::stopRotation()
{
  RCLCPP_INFO(this->get_logger(), "Stopped rotation");
  std_msgs::msg::Float64MultiArray cmdVel;
  cmdVel.data.emplace_back(0);
  mVelocityPublisher_->publish(cmdVel);
}

/**
 * @brief Implementation of the euclidean modulus
 *
 * @param num Dividend of the division
 * @param base Divisor of the division
 * @return Modulus in the euclidean sense (Like in Python)
 */
float CamMover::euclMod(float num, float base)
{
  return std::fmod((std::fmod(num, base) + base), base);
}

/**
 * @brief Simple timer for joint the camera joint velocity control
 */
void CamMover::velCallback()
{
  float diff = mTarget_ - euclMod(mCurrentJointPos_, 2 * M_PI);

  diff = euclMod(diff + M_PI, 2 * M_PI) - M_PI;

  diff = fmin(1, diff);
  diff = fmax(-1, diff);
  std_msgs::msg::Float64MultiArray cmdVel;
  cmdVel.data.emplace_back(0.95 * diff);
  mVelocityPublisher_->publish(cmdVel);
}
