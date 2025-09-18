"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the package share directory
    pkg_share = get_package_share_directory('sas_extended_kalman_filter_unitree_b1')
    
    # Construct the full path to the RViz config file
    rviz_config_path = os.path.join(pkg_share, 'launch', 'config', 'ekf_config.rviz')


    unitree_b1_white_vicon_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_extended_kalman_filter_unitree_b1'), 'launch'),
            '/unitree_b1_white_vicon_ekf_launch.py'])
    )  


    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
                            
        unitree_b1_white_vicon_ekf_launch,

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        )

    ])