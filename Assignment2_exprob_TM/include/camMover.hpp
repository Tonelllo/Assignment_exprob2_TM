#pragma once

#include "arucoDetector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CamMover : public rclcpp::Node
{
  ArucoDetector mArucoDetector_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mVelocityPublisher_;
  void getCurrentFrame(const sensor_msgs::msg::Image::ConstSharedPtr&,
                       const sensor_msgs::msg::JointState::ConstSharedPtr&);
  std::map<int, float> mDetectedIds_;
  size_t mCurrentSearchingIndex_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDetectionPublisher_;
  rclcpp::TimerBase::SharedPtr mTimer_;

  message_filters::Subscriber<sensor_msgs::msg::Image> mImageSubscriber_;
  message_filters::Subscriber<sensor_msgs::msg::JointState> mJointSubscriber_;
  using mSyncPolicy_ =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::JointState>;
  using mSync_ = message_filters::Synchronizer<mSyncPolicy_>;
  std::shared_ptr<mSync_> mSyncronizer_;
  void velCallback();
  float euclMod(float, float);
  std::atomic<float> mTarget_;
  std::atomic<float> mCurrentJointPos_;

public:
  CamMover();
};
