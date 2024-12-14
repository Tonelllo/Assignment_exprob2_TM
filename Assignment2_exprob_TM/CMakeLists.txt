cmake_minimum_required(VERSION 3.5)
project(assignment2_exprob_tm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(assignment2_exprob_tm src/assignment2_exprob_tm.cpp)
add_executable(move_action_node src/move_action_node.cpp)
add_executable(explore_waypoint_action_node src/explore_waypoint_action_node.cpp)
add_executable(mission_controller_node src/mission_controller_node.cpp)

ament_target_dependencies(assignment2_exprob_tm rclcpp geometry_msgs ros2_aruco_interfaces cv_bridge OpenCV sensor_msgs message_filters)

ament_target_dependencies(move_action_node
  geometry_msgs
  nav2_msgs
  plansys2_executor
  rclcpp
  rclcpp_action
)

ament_target_dependencies(explore_waypoint_action_node
  geometry_msgs
  nav2_msgs
  plansys2_executor
  rclcpp
  rclcpp_action
)

ament_target_dependencies(mission_controller_node
  plansys2_pddl_parser
  plansys2_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  rclcpp
  rclcpp_action
)

target_include_directories(assignment2_exprob_tm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
	assignment2_exprob_tm
	move_action_node
	explore_waypoint_action_node
	mission_controller_node
	
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch urdf config worlds models
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
