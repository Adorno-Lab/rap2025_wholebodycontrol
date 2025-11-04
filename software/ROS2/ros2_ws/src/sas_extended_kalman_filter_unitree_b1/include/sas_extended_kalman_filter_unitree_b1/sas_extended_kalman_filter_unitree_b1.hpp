/*
# Copyright (c) 2025 Adorno-Lab
#
#    This file is part of sas_robot_driver_unitree_z1.
#
#    sas_robot_driver_unitree_z1 is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_unitree_z1 is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_unitree_z1.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on sas_robot_driver_ur.hpp 
#   (https://github.com/MarinhoLab/sas_robot_driver_ur/blob/main/include/sas_robot_driver_ur/sas_robot_driver_ur.hpp)
#
# ################################################################*/

#pragma once
#include <atomic>

#include <thread>
#include <Eigen/Eigen>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_datalogger/sas_datalogger_client.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <dqrobotics/DQ.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace Eigen;
using namespace DQ_robotics;

namespace sas
{


struct ExtendedKalmanFilterConfiguration
{
    std::string topic_prefix;
    double thread_sampling_time_sec;
    std::string robot_vicon_marker_rear;
    std::string robot_vicon_marker_front;

};

class ExtendedKalmanFilter
{
private:

    std::atomic_bool* st_break_loops_;
    bool _should_shutdown() const;

    ExtendedKalmanFilterConfiguration configuration_;


    sas::Clock clock_;
    std::shared_ptr<rclcpp::Node> node_;
    sas::DataloggerClient datalogger_client_;
    bool save_data_with_datalogger_{false};

    //-------------------topics----------------------//
    /*
     * The UnitreeB1 driver provides the linear and angular velocity based on
     * the IMU data. Therefore, I'll use them instead of using raw IMU data.
     *
     */

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_IMU_state_;
    void _callback_subscriber_IMU_state(const sensor_msgs::msg::Imu& msg);
    bool new_IMU_data_available_{false};

    // DQ linear_acceleration_IMU_{0}; Not used
    // DQ angular_velocity_IMU_{0};  Not used
    DQ orientation_IMU_{1}; // Used to compute the orientation offset between the Vicon marker


    DQ x_offset_rear_to_central_body_frame_ = 1 + 0.5*E_*(0.25*i_ -0.17*k_); //0.15933

    DQ x_rear_to_front_{1};

    //-----------------------------------------
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_twist_state_;
    void _callback_subscriber_twist_state(const geometry_msgs::msg::TwistStamped& msg);
    bool new_twist_data_available_{false};
    DQ linear_velocity_from_robot_data_{0};
    DQ angular_velocity_from_robot_data_{0};



    //------------------------
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    void _try_update_vicon_markers();
    bool new_vicon_data_available_rear_{false};
    bool new_vicon_data_available_front_{false};
    unsigned int vicon_stamp_nano_rear_{0};
    unsigned int vicon_stamp_nano_front_{0};
    int data_loss_counter_rear_{0};
    int data_loss_counter_front_{0};
    const int DATA_LOSS_THRESHOLD_ = 10;
    DQ _geometry_msgs_msg_TransformStamped_to_dq(const geometry_msgs::msg::TransformStamped &msg);
    DQ vicon_pose_rear_{1};
    DQ vicon_pose_front_{1};
    DQ vicon_pose_{1};
    bool show_status_{true};


    //-------------------------
    double _compute_line_to_line_angle_distance(const DQ& vicon_pose_marker);
    void _set_height_from_marker(const DQ& x);
    //-----------------------


    DQ estimated_robot_pose_{1};
    DQ predicted_robot_pose_{1};
    double vicon_height_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_estimated_robot_marker_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_estimated_robot_marker_pose_with_offset_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_debug_;


    void _publish_pose_stamped(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
                               const std::string& frame_id,
                               const DQ& pose);

    void _prediction_step();
    void _update_step();


    double normalize_angle(double angle);
    VectorXd _get_mobile_platform_configuration_from_pose(const DQ& pose) const;
    MatrixXd jacobian_matrix(const double& v, const double& phi);
    //-----------------



    // State vector: [x, y, phi]
    Eigen::VectorXd x_;
    Eigen::MatrixXd SIGMA_; // State covariance (3x3)
    Eigen::MatrixXd R_; // Covariance related to randomness in the state transition

    Eigen::MatrixXd H_; // Observation matrix (3x3)

    Eigen::MatrixXd Q_; // Measurement noise covariance (3x3)


public:

    ExtendedKalmanFilter(const ExtendedKalmanFilter&)=delete;
    ExtendedKalmanFilter()=delete;
    ~ExtendedKalmanFilter();

    ExtendedKalmanFilter(std::shared_ptr<rclcpp::Node> &node,
                         const ExtendedKalmanFilterConfiguration &configuration,
                         std::atomic_bool* break_loops);

    void control_loop();

};



}
