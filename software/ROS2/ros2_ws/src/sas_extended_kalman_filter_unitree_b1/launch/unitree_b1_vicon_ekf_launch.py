"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the package share directory
    pkg_share = get_package_share_directory('sas_extended_kalman_filter_unitree_b1')
    
    # Construct the full path to the RViz config file
    rviz_config_path = os.path.join(pkg_share, 'launch', 'config', 'ekf_config.rviz')


    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='sas_extended_kalman_filter_unitree_b1',
            executable='sas_extended_kalman_filter_unitree_b1_node',
            name='ekf_b1_1',
            namespace="sas_b1",
            output="screen",
            parameters=[{
                "topic_prefix": "/sas_b1/b1_1",
                "thread_sampling_time_sec": 0.001,
                "robot_vicon_marker": "B1Z1_Frame_1"
            }]
        ),

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        )

    ])