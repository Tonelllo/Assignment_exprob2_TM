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

#include <iomanip>
#include <plansys2_pddl_parser/Utils.h>

#include <memory>

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
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp0)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(to_go wp3)"));
  }

  void show_progress() {
    auto feedback = executor_client_->getFeedBack();
    if (feedback.action_execution_status.size() != 8)
      return;
    std::cout << "\n";
    for (const auto &action_feedback : feedback.action_execution_status) {
      if (action_feedback.action == "move") {
        std::cout << "Move to " << action_feedback.arguments[1]
                  << " completed with percentage " << std::setprecision(2)
                  << action_feedback.completion * 100.0 << "%"
                  << std::setfill(' ') << std::setw(5) << std::endl;
      } else if (action_feedback.action == "explore_waypoint") {
        std::cout << "Exploring " << action_feedback.arguments[1]
                  << " completed with percentage " << std::setprecision(2)
                  << action_feedback.completion * 100.0 << "%"
                  << std::setfill(' ') << std::setw(5) << std::endl;
      }
    }
    std::cout << "\033[9F";
  }

  void step() {
    show_progress();
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
        std::cout << "Goal: " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
        }
      
      if (!plan.has_value()) {
        std::cout << "Could not find plan to reach goal "
                  << parser::pddl::toString(problem_expert_->getGoal())
                  << std::endl;
        break;
      }

      // Execute the plan
      if (executor_client_->start_plan_execution(plan.value())) {
        state_ = PATROL_WP;
      }
    } break;
    case PATROL_WP: {
      auto feedback = executor_client_->getFeedBack();

      if (!executor_client_->execute_and_check_plan() &&
          executor_client_->getResult()) {
        if (executor_client_->getResult().value().success) {
          std::cout << "Successful finished " << std::endl;

          // Check which waypoint has been explored to add predicate
          auto current_goal = problem_expert_->getGoal();
          std::string current_goal_str = parser::pddl::toString(current_goal);
          std::string current_waypoint;

          // Extract the waypoint from the current goal
          size_t start_pos = current_goal_str.find("wp");
          if (start_pos != std::string::npos) {
            size_t end_pos = current_goal_str.find(")", start_pos);
            current_waypoint = current_goal_str.substr(start_pos, end_pos - start_pos);
            std::cout << "Current goal waypoint: " << current_waypoint << std::endl;
            } else {
            std::cout << "No waypoint found in the current goal" << std::endl;
            }

          std::string explored_predicate = "(explored " + current_waypoint + ")";
          std::string to_go_predicate = "(to_go " + current_waypoint + ")";
          problem_expert_->addPredicate(plansys2::Predicate(explored_predicate));
          problem_expert_->removePredicate(plansys2::Predicate(to_go_predicate));

          // Set the goal for next state
          //problem_expert_->setGoal(plansys2::Goal("(and(explored wp1))")); SECONDO ME NON SERVE PERCHè SONO SETTATI ALL'INIZIO
          // Check if all goals are satisfied
          if (problem_expert_->isGoalSatisfied(problem_expert_->getGoal())) {  // DA VEDERE SE é GIUSTO QUELLO CHE HO INSERITO DENTRO A ISGOALSATISFIED
            state_ = FINISHED;
          } else {
            // Compute the plan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Could not find plan to reach goal "
                        << parser::pddl::toString(problem_expert_->getGoal())
                        << std::endl;
              break;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value())) {
              state_ = PATROL_WP;
            }}
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
    }break;

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
    PATROL_WP,
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
