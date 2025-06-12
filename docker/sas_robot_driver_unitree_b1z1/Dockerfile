FROM juanjqo/ubuntu_24_dqrobotics_ros_jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



# Install install_sas_drivers_b1z1 and build the ROS packages
RUN chmod +x ~/git/docker_recipes/dependencies/install_sas_drivers_b1z1.sh


RUN bash -c '~/git/docker_recipes/dependencies/install_sas_drivers_b1z1.sh' \
&& /bin/bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 


