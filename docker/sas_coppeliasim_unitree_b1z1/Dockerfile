FROM murilomarinho/sas:jazzy
ENV BASH_ENV="/etc/bash_env"

# For local tests using CoppeliaSim
ARG COPPELIASIM_RELEASE=V4_7_0_rev4
ENV COPPELIASIM_RELEASE=$COPPELIASIM_RELEASE

ARG UBUNTU_VERSION=Ubuntu24_04
ENV UBUNTU_VERSION=$UBUNTU_VERSION

ARG COPPELIASIM_FILE="CoppeliaSim_Edu_${COPPELIASIM_RELEASE}_${UBUNTU_VERSION}.tar.xz"
ENV COPPELIASIM_FILE=$COPPELIASIM_FILE

# Install CoppeliaSim. # This is temporal: && git reset --hard origin/master && git pull origin master
RUN cd ~/git/docker_recipes && git reset --hard origin/master && git pull origin master
RUN chmod +x ~/git/docker_recipes/dependencies/install_coppeliasim.sh
RUN bash -c '~/git/docker_recipes/dependencies/install_coppeliasim.sh'

# Clone this rep
RUN mkdir -p ~/git/
RUN cd ~/git/ && git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol --recursive 
RUN cd ~/git/rap2025_wholebodycontrol && git pull

# copy the folder ~/git/rap2025_wholebodycontrol/ to ~/ros2_ws/src/rap2025_wholebodycontrol/
RUN cp -r ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/ \
 ~/ros2_ws/src/ \
&& /bin/bash -c "source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws/; colcon build && source install/setup.bash" 

RUN chmod +x ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/scripts/launch_ROS2_drivers.sh  
RUN chmod +x ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/scripts/launch_scene.sh  
RUN chmod +x ~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/scripts/launch_scene_and_ROS2_drivers.sh

RUN echo "alias launch_ROS2_drivers='cdros2 && source install/setup.bash && ros2 launch sas_coppeliasim_unitree_b1z1 sas_coppeliasim_unitree_b1z1_launch.py ' " >> ~/.bashrc 
RUN echo "alias launch_scene='~/utils/CoppeliaSim_Edu_V4_7_0_rev4_Ubuntu24_04/coppeliaSim.sh -s 0 -f ~/git/rap2025_wholebodycontrol/coppeliasim_scenes/B1_with_Z1_showcase.ttt'" >> ~/.bashrc 


RUN echo "alias launch_scene_and_ROS2_drivers='~/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_coppeliasim_unitree_b1z1/scripts/launch_scene_and_ROS2_drivers.sh'" >> ~/.bashrc 




