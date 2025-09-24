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
            package='sas_coppeliasim_unitree_b1z1',
            executable='sas_b1z1_coppeliasim_node',
            name='b1z1_1',
            namespace="",
            output="screen",
            parameters=[{
                "cs_host": "localhost",
                "cs_port": 23000,
                "cs_TIMEOUT_IN_MILISECONDS": 8000,
                "cs_B1_robotname": "UnitreeB1_1",
                "robot_model_frame_name": "x1",
                "desired_frame_name": "xd1",
                "robot_marker_frame_name": "B1Z1_Frame_robot_white_rear",
                "thread_sampling_time_sec": 0.001,
                "B1_topic_prefix": "sas_b1/b1_1",
                "Z1_topic_prefix": "sas_z1/z1_1",
            }]
        ),

    ])
