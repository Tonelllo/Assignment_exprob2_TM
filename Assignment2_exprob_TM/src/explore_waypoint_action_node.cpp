
#include <math.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>

#include "arucoDetector.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
class ExploreWaypointAction : public plansys2::ActionExecutorClient {
public:
  ExploreWaypointAction()
      : plansys2::ActionExecutorClient("explore_waypoint", 1s) {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {
    progress_ = 0.0;
    searching = false;
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    cmd_vel_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  bool searching;
  ArucoDetector mArucoDetector_;
  int found_id = -1;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mCameraSubscriber_ =
      this->create_subscription<sensor_msgs::msg::Image>(
          "/camera/image_raw", 1,
          std::bind(&ExploreWaypointAction::getCurrentFrame, this,
                    std::placeholders::_1));
  void getCurrentFrame(const sensor_msgs::msg::Image::SharedPtr img) {
    mArucoDetector_.detect(img);
    cv::Mat &cf = mArucoDetector_.currentFrame_;

    // vvv COMMENT THIS SECTION TO NOT HAVE THE VISUALIZATION
    cv::Mat cf_copy;
    cf.copyTo(cf_copy);
    cv::aruco::drawDetectedMarkers(cf_copy, mArucoDetector_.markerCorners_,
                                   mArucoDetector_.markerIds_);
    cv::imshow("Detection Visualization", cf_copy);
    cv::waitKey(10);
    auto arr = mArucoDetector_.markerIds_;
    if (arr.size() > 0 && searching) {
      found_id = arr[0];
    }
  }
  void do_work() {
    searching = true;
    if (progress_ < 1.0) {

      send_feedback(progress_, "Exploring waypoint");

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.2;
      if (found_id > 0) {
        progress_ = 1.0;
        std::cout << found_id << std::endl;
      }
      cmd_vel_pub_->publish(cmd);
    } else {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = 0.0;

      cmd_vel_pub_->publish(cmd);

      searching = false;
      finish(true, 1.0, "Id: "+std::to_string(found_id));
      found_id = -1;
    }
  }

  float progress_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreWaypointAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "explore_waypoint"));
  node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}