/*
#    Copyright (c) 2024-2025 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

#pragma once
#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <sas_datalogger/sas_datalogger_client.hpp>


//using namespace Eigen;

using namespace rclcpp;

namespace sas
{

struct ControllerConfiguration
{
    std::string cs_host;
    int cs_port;
    int cs_TIMEOUT_IN_MILISECONDS;
    std::string cs_B1_robotname;
    std::string cs_Z1_robotname;
    std::string cs_desired_frame;
    std::string B1_topic_prefix;
    std::string Z1_topic_prefix;
    double thread_sampling_time_sec;
    std::string vfi_file;
    double controller_proportional_gain;
    double controller_damping;
    double controller_target_region_size;
    double controller_target_exit_size;
    bool controller_enable_parking_break_when_gripper_is_open;
    bool debug_wait_for_topics;
};

class B1Z1WholeBodyControl
{
protected:


    ControllerConfiguration configuration_;
    std::atomic_bool* st_break_loops_;

    std::shared_ptr<rclcpp::Node> node_;
    DQ robot_pose_; // pose of the B1 robot;
    VectorXd q_arm_;
    double target_gripper_position_{0};
    std::vector<std::string> z1_jointnames_;

    bool update_handbreak_{true};
    bool update_handbreak_released_{true};
    bool robot_reached_region_;



private:

    //double timer_period_;
    double T_;



    sas::Clock clock_;


    //also equivalent to rclcpp::TimerBase::SharedPtr
    std::shared_ptr<rclcpp::TimerBase> timer_;


    Subscription <sensor_msgs::msg::JointState>::SharedPtr subscriber_Z1_joint_states_;
    void _callback_Z1_joint_states(const sensor_msgs::msg::JointState& msg);

    bool new_robot_pose_data_available_;
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_state_;
    void _callback_pose_state(const geometry_msgs::msg::PoseStamped& msg);

    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_holonomic_velocities_;
    //Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_arm_positions_;


    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_gripper_position_from_coppeliasim_;
    void _callback_gripper_position_from_coppeliasim(const std_msgs::msg::Float64MultiArray& msg);


    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_frame_x_;


    DQ xd_;
    bool new_coppeliasim_xd_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_xd_;
    void _callback_xd_state(const geometry_msgs::msg::PoseStamped& msg);




    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;

protected:

    bool _should_shutdown() const;

    void _publish_target_B1_commands(const VectorXd& u_base_vel);
    void _publish_coppeliasim_frame_x(const DQ& pose);
    void _connect();
    void _update_kinematic_model();

    VectorXd _get_mobile_platform_configuration_from_pose(const DQ& pose) const;

    VectorXd _get_planar_joint_velocities_at_body_frame(const VectorXd& planar_joint_velocities_at_inertial_frame) const;

public:

    B1Z1WholeBodyControl(const B1Z1WholeBodyControl&)=delete;
    B1Z1WholeBodyControl()=delete;
    ~B1Z1WholeBodyControl();

    B1Z1WholeBodyControl(std::shared_ptr<Node>& node,
                         const  ControllerConfiguration &configuration,
                         std::atomic_bool* break_loops);


    void control_loop();

};



}
