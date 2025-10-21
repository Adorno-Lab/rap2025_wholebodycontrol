FROM murilomarinho/sas:jazzy
ENV BASH_ENV="/etc/bash_env"



# Clone this rep
RUN mkdir -p ~/git/
RUN cd ~/git/ && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol --recursive 
RUN cd ~/git/rap2025_wholebodycontrol && git pull


RUN chmod +x ~/git/rap2025_wholebodycontrol/dependencies/install_robot_constraint_manager.sh
RUN bash -c '~/git/rap2025_wholebodycontrol/dependencies/install_robot_constraint_manager.sh' 


# copy the folder ~/git/rap2025_wholebodycontrol/ to ~/ros2_ws/src/rap2025_wholebodycontrol/
RUN cp -r ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_control_unitree_b1z1/ \
 ~/ros2_ws/src/ \
&& /bin/bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 

RUN echo "alias launch_sas_control='cdros2 && source install/setup.bash && ros2 launch sas_control_unitree_b1z1 sas_control_unitree_b1z1_white_launch.py ' " >> /etc/bash_env
RUN echo "source /etc/bash_env" >> ~/.bashrc






