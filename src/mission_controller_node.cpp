#include <algorithm>
#include <climits>
#include <iomanip>
#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <rclcpp/utilities.hpp>
#include <regex>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

class MissionController : public rclcpp::Node {
public:
  std::atomic<bool> running = true;
  std::vector<std::string>
      visited_waypoints; // Ids of the waypoints as strings ex: "wp0"
  std::vector<uint>
      aruco_waypoints; // Ids of the aruco on the waypoints as uints ex: 12
  MissionController() : rclcpp::Node("mission_controller"), state_(STARTING) {}

  /**
   * @brief Initializes all the ojects needed to control the pddl planning
   */
  void init() {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  /**
   * @brief Initialization of the problem file for the planning
   *
   * The robot is in an unknown position at the beginning meaning that it's not
   * in a waypoint. Since this is set as an initial starting position or
   * recovery position it's already explored.
   *
   * (to_go) is used to force the planner to use the "move" action instead of
   * the move_to_min
   *
   * (explored) is used to indicate that the aruco node in that waypoint has
   * been identified
   *
   * All the waypoints and the robot get initialized.
   */
  void init_knowledge() {
    problem_expert_->addInstance(plansys2::Instance{"rob", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"unknown", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp0", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});

    problem_expert_->addPredicate(
        plansys2::Predicate("(at-robby rob unknown)"));
    problem_expert_->addPredicate(plansys2::Predicate("(explored unknown)"));
    vecPreds_ = {
        plansys2::Predicate("(to_go wp0)"), plansys2::Predicate("(to_go wp1)"),
        plansys2::Predicate("(to_go wp2)"), plansys2::Predicate("(to_go wp3)")};
    for (const auto &pred : vecPreds_) {
      problem_expert_->addPredicate(pred);
    }
  }

  /**
   * @brief Displays the progress for each waypoint. 
   *
   * Note that it's updating in
   * place so if the terminal is too small the prints will result incorrect
   */
  void show_progress() {
    auto feedback = executor_client_->getFeedBack();
    if (feedback.action_execution_status.empty())
      return;
    for (const auto &action_feedback : feedback.action_execution_status) {
      if (action_feedback.action == "move") {
        std::cout << "Move to " << action_feedback.arguments[2]
                  << " completed with percentage " << std::setprecision(3)
                  << action_feedback.completion * 100.0 << "%          "
                  << std::endl;
      } else if (action_feedback.action == "explore_waypoint") {
        std::cout << "Exploring " << action_feedback.arguments[1]
                  << " completed with percentage " << std::setprecision(3)
                  << action_feedback.completion * 100.0 << "%          "
                  << std::endl;
      } else if (action_feedback.action == "move_to_min") {
        std::cout << "Moving to min " << action_feedback.arguments[2]
                  << " completed with percentage " << std::setprecision(3)
                  << action_feedback.completion * 100.0 << "%          "
                  << std::endl;
      }
    }
    std::cout << "\033[" +
                     std::to_string(feedback.action_execution_status.size()) +
                     "F";
  }

  /**
   * @brief Gets the current state of the actions
   *
   * This to allow the update of the
   * problem for the second phase of going to the minimum valued aruco node
   */
  void get_progress() {
    auto feedback = executor_client_->getFeedBack();
    std::smatch m;
    std::regex r(R"(Id: (\w+))");
    for (const auto &action_feedback : feedback.action_execution_status) {
      if (action_feedback.action == "explore_waypoint") {
        auto waypoint = action_feedback.arguments[1];
        // Regex_match returns true whether the regex matches the string and in
        // the 1 position of the matches(smatch) there is the first
        // capturing group.
        // Once there is a match it means that the the node has been explored
        // because the success string is printed.
        if (std::regex_match(action_feedback.message_status, m, r)) {
          if (std::find(visited_waypoints.begin(), visited_waypoints.end(),
                        waypoint) == visited_waypoints.end()) {
            // If the waypoint is not already inserted in the visited_waypoints
            // vector it means that is the first time that we reach it and we
            // have found the aruco id since the regex has matches so we can
            // work on it.

            // Here we take the id of the aruco
            auto toInsert = std::stoi(m[1].str());

            // Here we insert into the aruco_waypoints the id while keeping the
            // ascending ordering thanks to the lower_bound function
            auto iter = std::lower_bound(aruco_waypoints.begin(),
                                         aruco_waypoints.end(), toInsert);
            // *VERY* Important that the distance is checked *BEFORE* the
            // insertion otherwise the iter gets invalidated since the new value
            // is inserted before it
            auto dist = std::distance(aruco_waypoints.begin(), iter);
            aruco_waypoints.insert(iter, toInsert);

            // Since visited_waypoints and aruco_waypoints are vector of
            // different types we cannot use the same iterator. In order to have
            // the two arrays behave like a map of ordered elements we use the
            // index of where the new aruco id has been inserted thanks to the
            // distance function.
            //
            // Once we have the distance that is the index we can insert at that
            // position the waypoint string in the visited_waypoints vector.
            // next is the same as doing visited_waypoints.begin() + dist, but
            // safer
            visited_waypoints.insert(std::next(visited_waypoints.begin(), dist),
                                     waypoint);
          }
        }
      }
    }
  }

  /**
   * @brief Function run for each cycle of the node
   *
   * This contains the state machine that controls the execution of the node in
   * combination with the get_progress function.
   */
  void step() {
    show_progress();
    get_progress();

    switch (state_) {
    case STARTING: {
      // Set the initial goal
      problem_expert_->setGoal(plansys2::Goal(
          "(and(explored wp0)(explored wp1)(explored wp2)(explored wp3))"));

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      // If the plan contains something
      if (plan.has_value()) {
        std::cout << "Plan found:" << std::endl;
        for (const auto &action : plan.value().items) {
          std::cout << action.action << " [";
        }
        std::cout << "Goal: "
                  << parser::pddl::toString(problem_expert_->getGoal())
                  << std::endl;
      }else if(!plan.has_value()) {
        std::cout << "Could not find plan to reach goal "
                  << parser::pddl::toString(problem_expert_->getGoal())
                  << std::endl;
        break;
      }

      // Execute the plan
      if (executor_client_->start_plan_execution(plan.value())) {
        state_ = EXPLORE_WP;
      }
    } break;

    case EXPLORE_WP: {
      auto feedback = executor_client_->getFeedBack();
      if (!executor_client_->execute_and_check_plan() &&
          executor_client_->getResult()) {
        // Check which waypoint has been explored to add predicate
        if (executor_client_->getResult().value().success) {
          std::cout << "Successful finished " << std::endl;

          // If the goal is satisfied then we can go in the finish state
          if (problem_expert_->isGoalSatisfied(problem_expert_->getGoal())) {
            state_ = FINISHED_EXPLORING;
          } else {
            std::cout << "MAIN PLAN FAILED" << std::endl;
          }
        } else {
          for (const auto &action_feedback : feedback.action_execution_status) {
            if (action_feedback.status ==
                plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
              std::cout << "[" << action_feedback.action
                        << "] finished with error: "
                        << action_feedback.message_status << std::endl;
            }
          }

          auto pe = problem_expert_->getPredicates();
          for (const auto &pred : pe) {
            if (pred.name == "at-robby") {
              problem_expert_->removePredicate(pred);
            }
          }
          problem_expert_->addPredicate(
              plansys2::Predicate("(at-robby rob unknown)"));
          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Unsuccessful replan attempt to reach goal "
                      << parser::pddl::toString(problem_expert_->getGoal())
                      << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }
    } break;

    case FINISHED_EXPLORING: {
      // Get some space to not print over the previous showed data.
      std::cout << "\n\n\n\n\n\n\n\n\n\n\n";

      // Print the waypoints with the corresponding aruco node ids
      for (size_t i = 0; i < aruco_waypoints.size() - 1; i++) {
        std::cout << aruco_waypoints[i] << "\t" << visited_waypoints[i]
                  << std::endl;
      }
      std::cout << aruco_waypoints[aruco_waypoints.size() - 1] << "\t"
                << visited_waypoints[aruco_waypoints.size() - 1] << std::endl;

      auto pe = problem_expert_->getPredicates();
      for (const auto &pred : pe) {
        if (pred.name == "at-robby") {
          problem_expert_->removePredicate(pred);
        }
      }

      problem_expert_->addPredicate(
          plansys2::Predicate("(at-robby rob unknown)"));
      for (const auto &pred : vecPreds_) {
        auto waypointName = pred.parameters[0].name;
        // "wp2"
        if (waypointName.find(visited_waypoints[0]) != std::string::npos) {
          problem_expert_->removePredicate(pred);
          problem_expert_->setGoal(
              plansys2::Goal("(and(min " + waypointName + "))"));
          break;
        }
      }

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      std::cout << problem << std::endl;

      if (plan.has_value()) {
        std::cout << "Second plan found:" << std::endl;
        for (const auto &action : plan.value().items) {
          std::cout << action.action << " [";
        }
        std::cout << "Goal: "
                  << parser::pddl::toString(problem_expert_->getGoal())
                  << std::endl;
      }

      if (!plan.has_value()) {
        std::cout << "Could not find plan to reach goal "
                  << parser::pddl::toString(problem_expert_->getGoal())
                  << std::endl;
        break;
      }

      // Execute the plan
      if (executor_client_->start_plan_execution(plan.value())) {
        state_ = GO_TO_SMALLEST;
      }
    } break;

    case GO_TO_SMALLEST: {
      auto feedback = executor_client_->getFeedBack();
      if (!executor_client_->execute_and_check_plan() &&
          executor_client_->getResult()) {
        // Check which waypoint has been explored to add predicate
        if (executor_client_->getResult().value().success) {
          std::cout << "Successful finished " << std::endl;

          // Check if all goals are satisfied
          if (problem_expert_->isGoalSatisfied(
                  problem_expert_
                      ->getGoal())) { 
            state_ = FINISHED;
          } else {
            std::cout << "MAIN PLAN FAILED" << std::endl;
          }
        } else {
          for (const auto &action_feedback : feedback.action_execution_status) {
            if (action_feedback.status ==
                plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
              std::cout << "[" << action_feedback.action
                        << "] finished with error: "
                        << action_feedback.message_status << std::endl;
            }
          }

          // Replan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Unsuccessful replan attempt to reach goal "
                      << parser::pddl::toString(problem_expert_->getGoal())
                      << std::endl;
            break;
          }

          // Execute the plan
          executor_client_->start_plan_execution(plan.value());
        }
      }

    } break;

    case FINISHED: {
      std::cout << "Mission finished" << std::endl;
      running = false;
    }
    default:
      break;
    }
  }

private:
  std::vector<plansys2::Predicate> vecPreds_;
  typedef enum {
    STARTING,
    EXPLORE_WP,
    FINISHED_EXPLORING,
    GO_TO_SMALLEST,
    FINISHED
  } StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionController>();

  node->init();

  rclcpp::Rate rate(5);
  while (node->running && rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
