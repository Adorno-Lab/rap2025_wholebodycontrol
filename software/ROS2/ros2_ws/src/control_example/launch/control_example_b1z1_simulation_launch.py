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
            namespace="",
            output='screen',  
            emulate_tty=True,  
            parameters=[{
                "cs_host": "localhost",
                "cs_port": 23000,
                "cs_TIMEOUT_IN_MILISECONDS": 5000,
                "cs_B1_robotname": "UnitreeB1_1",
                "cs_Z1_robotname": "UnitreeZ1",
                "cs_desired_frame": "xd1",
                "B1_topic_prefix": "sas_b1/b1_1",
                "Z1_topic_prefix": "sas_z1/z1_1",
                "thread_sampling_time_sec": 0.002,
                "controller_proportional_gain": 8.0,
                "controller_damping": 0.1,
                "controller_target_region_size": 0.15,
                "controller_target_exit_size": 0.2,

                #"vfi_file":"/home/juanjqo/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/control_example/cfg/vfi_config.yaml",
                "vfi_file":"/home/s55322jq/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/control_example/cfg/vfi_config.yaml",

                
                "mobile_base_configuration_limits_min": [float('-inf'), float('-inf'), float('-inf'), float('-inf'), float('-inf'), float('-inf')],
                "mobile_base_configuration_limits_max": [float('inf'),   float('inf'), float('inf'),   float('inf'), float('inf'),  float('inf')],
                "mobile_base_configuration_velocity_limits_min": [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1],
                "mobile_base_configuration_velocity_limits_max": [ 0.1,  0.1,  0.1,  0.1,  0.1,  0.1],
                "arm_configuration_limits_min": [-30.0,  45.0, -90.0, -80.0, -45.0, -160.0], # degrees
                "arm_configuration_limits_max": [ 30.0, 165.0,   0.0,  80.0,  45.0,  160.0], # degrees
                "arm_configuration_velocity_limits_min": [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57], # rad/s
                "arm_configuration_velocity_limits_max": [ 1.57,  1.57,  1.57,  1.57,  1.57,  1.57], # rad/s
            }]
        ),

    ])
