11/21 simulation environment

Ubuntu 24.04

ROS 2 Jazzy


Packages:

navigation2

nav2_bringup

slam_toolbox

robot_localization

ros2_control

ros2_controllers

xacro

behaviortree_cpp

ros_gz


Build:

source /opt/ros/jazzy/setup.bash

colcon build



Run:

ros2 launch my_robot_sim sim_nav2.launch.py

ros2 run slope_bt_nodes test_bt
