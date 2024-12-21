- [Assumptions](#assumptions)
- [Installation](#installation)
  * [In an already present workspace](#in-an-already-present-workspace)
  * [In a new workspace](#in-a-new-workspace)
- [Documentation](#documentation)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# Assumptions
+ [Ros2 foxy](https://docs.ros.org/en/foxy/index.html) should be installed in the system
+ Using [ubuntu 20](https://releases.ubuntu.com/focal/)
# Installation
## In an already present workspace
+ Clone this repo inside the src folder of a ros2 foxy workspace
+ build the workspace
+ source the project
+ `ros2 launch exprob_dt assignment1.launch.py`
## In a new workspace
+ `mkdir -p MyWorkspace/src`
+ `cd MyWorkspace/src`
+ `git clone https://github.com/Tonelllo/exprob1_DT.git`
+ `cd ..`
+ `source /opt/ros/foxy/setup.bash`
+ `colcon build`
+ `source install/setup.bash`
+ `ros2 launch exprob_dt assignment1.launch.py`
# Documentation
You can find the documentation at [https://tonelllo.github.io/exprob1_DT/html/](https://tonelllo.github.io/exprob1_DT/html/)



ros2 launch slam_toolbox online_async_launch.py params_file:=/home/tonello/Exprob2/src/Assignment2_exprob_TM/config/slam_toolbox.yaml
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/tonello/Exprob2/src/Assignment2_exprob_TM/config/nav2.yaml
