/*
# Copyright (c) 2024 Juan Jose Quiroz Omana
#
#    This file is part of sas_robot_driver_unitree_b1.
#
#    sas_robot_driver_kuka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_kuka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_kuka.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on https://ros2-tutorial.readthedocs.io/en/latest/cpp/cpp_node.html
#
# ################################################################*/

#pragma once
#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>

#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


//using namespace Eigen;

using namespace rclcpp;

namespace sas
{

struct RobotDriverB1Z1CoppeliaSimConfiguration
{
    std::string cs_host;
    int cs_port;
    int cs_TIMEOUT_IN_MILISECONDS;
    std::string cs_B1_robotname;
    //std::string cs_Z1_robotname;
    std::string B1_topic_prefix;
    std::string Z1_topic_prefix;
    double thread_sampling_time_sec;
    std::string robot_model_frame_name;
    std::string desired_frame_name;
    std::string robot_marker_frame_name;

};

class B1Z1CoppeliaSimROS
{
private:
    std::atomic_bool* st_break_loops_;
    //std::string topic_prefix_b1_;
    //std::string topic_prefix_z1_;

    RobotDriverB1Z1CoppeliaSimConfiguration  configuration_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::vector<std::string> arm_jointnames_;
    std::string b1_name_;
    std::string z1_name_;
    std::string gripper_joint_name_;

    std::vector<std::string> FR_jointnames_;
    std::vector<std::string> FL_jointnames_;
    std::vector<std::string> RR_jointnames_;
    std::vector<std::string> RL_jointnames_;

    std::string FR_hip_rotor_;
    std::string FL_hip_rotor_;
    std::string RR_hip_rotor_;
    std::string RL_hip_rotor_;


private:
    int print_count_;

    sas::Clock clock_;

    //also equivalent to rclcpp::TimerBase::SharedPtr
    std::shared_ptr<rclcpp::TimerBase> timer_;


    VectorXd qFR_cmd_;
    VectorXd qFL_cmd_;
    VectorXd qRR_cmd_;
    VectorXd qRL_cmd_;
    VectorXd q_arm_cmd_;

    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_FR_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_FL_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_RR_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_RL_joint_states_;
    Subscription <sensor_msgs::msg::JointState>::SharedPtr subscriber_Z1_joint_states_;

    void _callback_FR_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_FL_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_RR_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_RL_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_Z1_joint_states(const sensor_msgs::msg::JointState& msg);

    //DQ IMU_pose_{1};
    DQ robot_pose_{1};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_state_;
    void _callback_pose_state(const geometry_msgs::msg::PoseStamped& msg);
    bool new_robot_pose_data_available_{false};

    DQ robot_marker_{1};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_robot_marker_;
    void _callback_robot_marker(const geometry_msgs::msg::PoseStamped& msg);






    DQ x_fkm_{1};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_x_fkm_;
    void _callback_x_fkm_state(const geometry_msgs::msg::PoseStamped& msg);


    DQ B1Z1_Frame_1_previous_{1};

    DQ UnitreeB1_initial_pose_{1};
    DQ B1Z1_Frame_1_offset_{1};
    DQ x_IMU_orientation_offset_{1};



    double gripper_position_{0};



    DQ geometry_msgs_msg_TransformStamped_to_dq(const geometry_msgs::msg::TransformStamped& msg);




    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_frame_xd_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_gripper_position_from_coppeliasim_;



    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::string> vicon_markers_ = {"B1Z1_Frame_0", "B1Z1_Frame_1", "B1Z1_Frame_2", "B1Z1_Frame_3"};
    std::tuple<bool, DQ> _try_get_vicon_marker(const std::string& marker_name);



private:

    void _set_joint_states_on_coppeliasim();
    void _set_robot_pose_on_coppeliasim(const DQ& pose);
    void _read_xd_and_publish();
    bool _should_shutdown() const;

   // DQ _compute_robot_pose_from_IMU_and_markers();


    void _publish_gripper_position();

    void _connect();


public:

    B1Z1CoppeliaSimROS(const B1Z1CoppeliaSimROS&)=delete;
    B1Z1CoppeliaSimROS()=delete;
    ~B1Z1CoppeliaSimROS();

    B1Z1CoppeliaSimROS(std::shared_ptr<Node>& node,
                        const RobotDriverB1Z1CoppeliaSimConfiguration &configuration,
                        std::atomic_bool* break_loops);
    void control_loop();

};



}
