#include <math.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient {
public:
  /**
   * @brief Constructor that initializes all the waypoints
   */
  MoveAction() : plansys2::ActionExecutorClient("move", 500ms) {
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "/map";
    wp.header.stamp = now();
    wp.pose.position.x = -7.0;
    wp.pose.position.y = -1.5;
    wp.pose.position.z = 0.0;
    wp.pose.orientation.x = 0.0;
    wp.pose.orientation.y = 0.0;
    wp.pose.orientation.z = 0.0;
    wp.pose.orientation.w = 1.0;
    waypoints_["wp0"] = wp;

    wp.pose.position.x = -3.0;
    wp.pose.position.y = -8.0;
    waypoints_["wp1"] = wp;

    wp.pose.position.x = 6.0;
    wp.pose.position.y = 2.0;
    waypoints_["wp2"] = wp;

    wp.pose.position.x = 7.0;
    wp.pose.position.y = -5.0;
    waypoints_["wp3"] = wp;

    using namespace std::placeholders;
    pos_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&MoveAction::current_pos_callback, this, _1));
  }

  /**
   * @brief Gets the curret position of the robot
   *
   * @param msg The position given by the amcl node
   */
  void current_pos_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pos_ = msg->pose.pose;
  }

  /**
   * @brief Sets up the functions to allow updates of the running state of the node
   *
   * @param previous_state The previous state in which the node was
   * @return The new state
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {
    send_feedback(0.0, "Move starting");

    navigation_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            shared_from_this(), "navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
          navigation_action_client_->wait_for_action_server(
              std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate =
        get_arguments()[2]; // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]",
                wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options = rclcpp_action::Client<
        nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this](NavigationGoalHandle::SharedPtr, NavigationFeedback feedback) {
          send_feedback( // NOTE that 0.5 is the allowed error
              std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining /
                                                 dist_to_move))),
              "Move running");
        };

    send_goal_options.result_callback = [this](auto) {
      finish(true, 1.0, "Move completed");
    };

    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(
        navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  /**
   * @brief Returns the distance between two points
   *
   * @param pos1 First Pose
   * @param pos2 Second Pose
   * @return returns the distance as a double
   */
  double getDistance(const geometry_msgs::msg::Pose &pos1,
                     const geometry_msgs::msg::Pose &pos2) {
    return sqrt((pos1.position.x - pos2.position.x) *
                    (pos1.position.x - pos2.position.x) +
                (pos1.position.y - pos2.position.y) *
                    (pos1.position.y - pos2.position.y));
  }

  void do_work() {}

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr>
      future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
