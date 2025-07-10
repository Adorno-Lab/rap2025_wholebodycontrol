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
            package='sas_control_unitree_b1z1',
            executable='sas_control_unitree_b1z1_node',
            name='b1z1_1',
            namespace="",
            parameters=[{
                "cs_host": "localhost",
                "cs_port": 23000,
                "cs_TIMEOUT_IN_MILISECONDS": 2000,
                "cs_B1_robotname": "UnitreeB1",
                "cs_Z1_robotname": "UnitreeZ1",
                "vfi_file":"/root/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/sas_control_unitree_b1z1/cfg/rcm_vfi_constraints.yaml",
                "B1_topic_prefix": "sas_b1/b1_1",
                "Z1_topic_prefix": "sas_z1/z1_1",
                "thread_sampling_time_sec": 0.002,
                "controller_proportional_gain": 8.0,
                "controller_damping": 0.05,
                "controller_target_region_size": 0.1,
                "controller_target_exit_size":, 0.26
                "debug_wait_for_topics": True
            }]
        ),

    ])
