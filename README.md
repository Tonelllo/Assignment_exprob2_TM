# Table of Content
- [Assumptions](#assumptions)
- [Installation](#installation)
  * [In an already present workspace](#in-an-already-present-workspace)
  * [In a new workspace](#in-a-new-workspace)
- [Documentation](#documentation)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

# In the docker container
+ `apt install ros-foxy-nav2*`
+ `apt install ros-foxy-plansys2-*`
+ `apt install ros-foxy-slam-toolbox*`
# Assumptions
+ [Ros2 foxy](https://docs.ros.org/en/foxy/index.html) should be installed in the system
+ Using [ubuntu 20](https://releases.ubuntu.com/focal/)
# Installation
## In an already present workspace
+ Clone this repo inside the src folder of a ros2 foxy workspace
+ build the workspace
+ source the project
+ `ros2 launch assignment2_exprob_tm assignment2.launch.py`
+ `ros2 run assignment2_exprob_tm mission_controller_node`
## In a new workspace
+ `mkdir -p MyWorkspace/src`
+ `cd MyWorkspace/src`
+ `git clone https://github.com/Tonelllo/Assignment_exprob2_TM.git`
+ `cd ..`
+ `source /opt/ros/foxy/setup.bash`
+ `colcon build`
+ `source install/setup.bash`
+ `ros2 launch assignment2_exprob_tm assignment2.launch.py`
+ `ros2 run assignment2_exprob_tm mission_controller_node`
# Documentation
You can find the documentation at [https://tonelllo.github.io/Assignment_exprob2_TM/html/](https://tonelllo.github.io/Assignment_exprob2_TM/html/)
# Notes
**NOTE** that the launch file in particular in the first runs is quite finicky. Please be patient and if the node does not start run again both the launchfile and the mission_manager node. 
