#!/bin/bash


cd ~/ros2_ws/ && colcon build
source ~/ros2_ws/install/setup.bash 
ros2 launch sas_coppeliasim_unitree_b1z1 sas_coppeliasim_unitree_b1z1_white_launch.py