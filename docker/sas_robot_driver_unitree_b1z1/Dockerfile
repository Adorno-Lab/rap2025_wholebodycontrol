FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"



# Clone this rep
RUN mkdir -p ~/git/
RUN cd ~/git/ && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol --recursive

RUN chmod +x ~/git/rap2025_wholebodycontrol/dependencies/install_sas_drivers_b1z1.sh
RUN bash -c '~/git/rap2025_wholebodycontrol/dependencies/install_sas_drivers_b1z1.sh' \
 ~/ros2_ws/src/ \
&& /bin/bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 


