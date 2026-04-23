FROM ghcr.io/adorno-lab/sas_unitree_b1z1_jazzy:latest  
ENV BASH_ENV="/etc/bash_env"

# Update SAS
RUN apt update && apt-get install ros-jazzy-sas-* -y

# Install QT
#RUN apt update && apt-get install qt6-base-dev qt6-tools-dev-tools -y


# Install robot_constraint_editor
RUN cd ~/git && git clone https://github.com/Adorno-Lab/robot_constraint_editor
RUN cd ~/git/robot_constraint_editor && mkdir -p build && cd build && cmake .. && make && make install



# Clone the robot_constraint_manager package
RUN cd ~/git && git clone https://github.com/Adorno-Lab/robot_constraint_manager 
RUN cd ~/git/robot_constraint_manager && mkdir -p build && cd build && cmake .. && make && make install




WORKDIR /app

RUN mkdir -p /app/software/ROS2/ros2_ws/src


# Copy files from this repository to the image
# Note: The folder sas_extended_kalman_filter_unitree_b1 is already included in the base image

COPY ./software/ROS2/ros2_ws/src/sas_control_unitree_b1z1 /app/software/ROS2/ros2_ws/src/sas_control_unitree_b1z1
COPY ./software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1 /app/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1 
COPY ./software/submodules /app/software/submodules






# copy the folder ~/git/rap2025_wholebodycontrol/ to ~/ros2_ws/src/rap2025_wholebodycontrol/
RUN cd /app/software/ROS2/ros2_ws/\
&& /bin/bash -c "source /opt/ros/jazzy/setup.bash; colcon build && source install/setup.bash" 

RUN echo "alias launch_sas_white_control='cdros2 && source install/setup.bash && ros2 launch sas_control_unitree_b1z1 sas_control_unitree_b1z1_white_launch.py ' " >> /etc/bash_env
RUN echo "alias launch_sas_black_control='cdros2 && source install/setup.bash && ros2 launch sas_control_unitree_b1z1 sas_control_unitree_b1z1_black_launch.py ' " >> /etc/bash_env



RUN ldconfig
RUN echo "source /etc/bash_env" >> ~/.bashrc






