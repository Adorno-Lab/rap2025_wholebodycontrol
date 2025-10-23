#!/bin/bash


cd ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/scripts
~/utils/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu24_04/coppeliaSim.sh -s 0 -f ~/git/rap2025_wholebodycontrol/coppeliasim_scenes/B1_with_Z1_showcase.ttt &

#read -p "If CoppeliaSim is ready, press enter to continue"

cd ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/scripts
./launch_ROS2_drivers.sh


