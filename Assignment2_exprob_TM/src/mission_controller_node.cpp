// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <climits>
#include <iomanip>
#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <regex>
#include <unordered_set>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MissionController : public rclcpp::Node {
public:
  std::vector<std::string> visited_waypoints;
  std::vector<uint> aruco_waypoints;
  MissionController() : rclcpp::Node("mission_controller"), state_(STARTING) {}

  void init() {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge() {
    problem_expert_->addInstance(plansys2::Instance{"rob", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"base", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp0", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(at-robby rob base)"));
    problem_expert_->addPredicate(plansys2::Predicate("(explored base)"));
    problem_expert_->addPredicate(plansys2::Predicate("(patrolled base)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp0)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp3)"));
  }

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
      } else if (action_feedback.action == "move_with_order") {
        std::cout << "Moving with order " << action_feedback.arguments[2]
                  << " completed with percentage " << std::setprecision(3)
                  << action_feedback.completion * 100.0 << "%          "
                  << std::endl;
      }
    }
    std::cout << "\033[" +
                     std::to_string(feedback.action_execution_status.size()) +
                     "F";
  }
  void get_progress() {
    auto feedback = executor_client_->getFeedBack();
    std::smatch m;
    std::regex r(R"(Id: (\w+))");
    for (const auto &action_feedback : feedback.action_execution_status) {
      if (action_feedback.action == "explore_waypoint") {
        auto waypoint = action_feedback.arguments[1];
        // Regex_match returns true whether the regex matches the string and in
        // the 1 position of the of the matches(smatch) there is the first
        // captuing group.
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

  void step() {
    show_progress();
    get_progress();
    switch (state_) {
    case STARTING: {
      // Set the goal for next state
      problem_expert_->setGoal(plansys2::Goal(
          "(and(explored wp0)(explored wp1)(explored wp2)(explored wp3))"));

      // Compute the plan
      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);

      if (plan.has_value()) {
        std::cout << "Plan found:" << std::endl;
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

          // Set the goal for next state
          // problem_expert_->setGoal(plansys2::Goal("(and(explored wp1))"));
          // SECONDO ME NON SERVE PERCHè SONO SETTATI ALL'INIZIO
          // Check if all goals are satisfied
          if (problem_expert_->isGoalSatisfied(
                  problem_expert_
                      ->getGoal())) { // DA VEDERE SE é GIUSTO QUELLO CHE HO
                                      // INSERITO DENTRO A ISGOALSATISFIED
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
              plansys2::Predicate("(at-robby rob base)"));
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
      std::cout << "\n\n\n\n\n\n\n\n\n\n\n";
      problem_expert_->addPredicate(plansys2::Predicate(
          "(comes_after base " + visited_waypoints[0] + ")"));
      for (size_t i = 0; i < aruco_waypoints.size() - 1; i++) {
        problem_expert_->addPredicate(
            plansys2::Predicate("(comes_after " + visited_waypoints[i] + " " +
                                visited_waypoints[i + 1] + ")"));
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
      problem_expert_->addPredicate(plansys2::Predicate("(at-robby rob base)"));
      problem_expert_->clearGoal();
      problem_expert_->setGoal(plansys2::Goal("(and"
                                              "(explored wp0)"
                                              "(explored wp1)"
                                              "(explored wp2)"
                                              "(explored wp3)"
                                              "(patrolled wp0)"
                                              "(patrolled wp1)"
                                              "(patrolled wp2)"
                                              "(patrolled wp3)"
                                              ")"));
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
        state_ = ORDERED_PATROL;
      }
    } break;
    case ORDERED_PATROL: {
      auto feedback = executor_client_->getFeedBack();
      if (!executor_client_->execute_and_check_plan() &&
          executor_client_->getResult()) {
        // Check which waypoint has been explored to add predicate
        if (executor_client_->getResult().value().success) {
          std::cout << "Successful finished " << std::endl;

          // Set the goal for next state
          // problem_expert_->setGoal(plansys2::Goal("(and(explored wp1))"));
          // SECONDO ME NON SERVE PERCHè SONO SETTATI ALL'INIZIO
          // Check if all goals are satisfied
          if (problem_expert_->isGoalSatisfied(
                  problem_expert_
                      ->getGoal())) { // DA VEDERE SE é GIUSTO QUELLO CHE HO
                                      // INSERITO DENTRO A ISGOALSATISFIED
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

          problem_expert_->addPredicate(
              plansys2::Predicate("(at-robby rob base)"));
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
    }
    default:
      break;
    }
  }

private:
  typedef enum {
    STARTING,
    EXPLORE_WP,
    FINISHED_EXPLORING,
    ORDERED_PATROL,
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
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
