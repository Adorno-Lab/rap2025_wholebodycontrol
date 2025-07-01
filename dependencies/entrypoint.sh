#!/bin/bash



source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/ && colcon build
source ~/ros2_ws/install/setup.bash
source ~/.bashrc
ros2 launch sas_robot_driver_unitree_b1 real_b1z1_robot_launch.py 
