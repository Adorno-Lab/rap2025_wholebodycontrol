FROM murilomarinho/sas_ros_jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



# Clone this rep
RUN mkdir -p ~/git/
RUN cd ~/git/ && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol --recursive 
RUN cd ~/git/rap2025_wholebodycontrol && git pull

# copy the folder ~/git/rap2025_wholebodycontrol/ to ~/ros2_ws/src/rap2025_wholebodycontrol/
RUN cp -r ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/ \
 ~/ros2_ws/src/ \
&& /bin/bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 

RUN echo "alias launch_coppeliasim='cdros2 && source install/setup.bash && ros2 launch sas_coppeliasim_unitree_b1z1 sas_coppeliasim_unitree_b1z1_launch.py ' " >> ~/.bashrc 



