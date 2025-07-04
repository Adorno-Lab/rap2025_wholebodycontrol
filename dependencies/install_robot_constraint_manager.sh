#!/bin/bash

mkdir -p ~/git 

# Clone the robot_constraint_manager package
cd ~/git && git git clone https://github.com/Adorno-Lab/robot_constraint_manager
cd ~/git/robot_constraint_manager
mkdir -p build 
cd ~/git/robot_constraint_manager/build && cmake .. && make && make install
ldconfig
