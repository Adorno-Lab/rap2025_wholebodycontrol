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
            package='control_example',
            executable='control_example_node',
            name='b1z1_1',
            output='screen',  # Output to screen (enables cout)
            emulate_tty=True,  # Emulate terminal for better output
            namespace="",
            parameters=[{
                "cs_host": "192.168.8.100",
                "cs_port": 23000,
                "cs_TIMEOUT_IN_MILISECONDS": 5000,
                "vfi_file":"/home/s55322jq/git/project-clerice/software/ROS2/ros2_ws/src/cff_model_tests/control_example/cfg/vfi_config.yaml",
                "cs_B1_robotname": "UnitreeB1_1",
                "cs_Z1_robotname": "UnitreeZ1",
                "cs_desired_frame": "xd1",
                "B1_topic_prefix": "sas_b1/b1_1",
                "Z1_topic_prefix": "sas_z1/z1_1",
                "thread_sampling_time_sec": 0.002,
                "configuration_limits_min_deg": [-10000.0, -10000.0, -10000.0, -30.0,  45.0,  -90.0, -80.0, -45.0, -160.0],  
                "configuration_limits_max_deg": [ 10000.0,  10000.0,  10000.0,  30.0, 165.0,    0.0,  80.0,  45.0,  160.0],  
                "config_vel_limits_min": [-0.15, -0.15, -0.15, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57],
                "config_vel_limits_max": [ 0.2,  0.15,  0.15,  1.57,  1.57,  1.57,  1.57,  1.57,  1.57],
            }]
        ),

    ])
