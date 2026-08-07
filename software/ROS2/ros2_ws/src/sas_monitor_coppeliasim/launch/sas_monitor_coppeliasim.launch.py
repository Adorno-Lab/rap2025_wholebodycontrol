"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                    'sigterm_timeout',
                    default_value='30'
                ),
        Node(
            package='sas_monitor_coppeliasim',
            executable='sas_monitor_coppeliasim_node',
            name='sas_monitor_coppeliasim',
            namespace="",
            output="screen",
            parameters=[{
                "cs_host": "192.168.8.100",
                "cs_port": 23000,
                "cs_TIMEOUT_IN_MILISECONDS": 8000,
                "cs_B1_1_robotname": "UnitreeB1_1/trunk_respondable",
                "cs_B1_2_robotname": "UnitreeB1_2/trunk_respondable",
                "B1_1_topic_prefix": "sas_b1/b1_1",
                "Z1_1_topic_prefix": "sas_z1/z1_1",
                "B1_2_topic_prefix": "sas_b1/b1_2",
                "Z1_2_topic_prefix": "sas_z1/z1_2",
                "thread_sampling_time_sec": 0.01,
                "cs_B1Z1_1_frame_x": "x1",
                "cs_B1Z1_2_frame_x": "x2",
                "cs_B1Z1_1_frame_xd": "xd1",
                "cs_B1Z1_2_frame_xd": "xd2",
                "obstacle_vicon_marker_name": "B1Z1_Frame_human_obstacle",#"B1Z1_Frame_0",
                "obstacle_coppeliasim_name": "Human_Cylinder"
               
            }]
        ),

    ])
