/*
# Copyright (c) 2025 Juan Jose Quiroz Omana
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
#
# ################################################################*/
#pragma once
#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <sas_core/sas_clock.hpp>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>


using namespace rclcpp;

namespace sas
{

struct MonitorCoppeliaSimConfiguration
{
    std::string cs_host;
    int cs_port;
    int cs_TIMEOUT_IN_MILISECONDS;
    std::string cs_B1_1_robotname;
    //std::string cs_Z1_1_robotname;
    std::string cs_B1_2_robotname;
    //std::string cs_Z1_2_robotname;

    std::string B1_1_topic_prefix;
    std::string Z1_1_topic_prefix;
    std::string B1_2_topic_prefix;
    std::string Z1_2_topic_prefix;


    double thread_sampling_time_sec;
    std::string cs_B1Z1_1_frame_x;
    std::string cs_B1Z1_2_frame_x;
    std::string cs_B1Z1_1_frame_xd;
    std::string cs_B1Z1_2_frame_xd;

};

class MonitorCoppeliaSim
{

private:
    std::atomic_bool* st_break_loops_;
    MonitorCoppeliaSimConfiguration  configuration_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    sas::Clock clock_;


    std::shared_ptr<UnitreeB1Z1RobotClient> robot_client_1_;


    std::vector<std::string> Z1_1_arm_jointnames_;

    //std::string b1_name_;


    std::vector<std::string> B1_1_FR_jointnames_;
    std::vector<std::string> B1_1_FL_jointnames_;
    std::vector<std::string> B1_1_RR_jointnames_;
    std::vector<std::string> B1_1_RL_jointnames_;



    std::string Z1_1_name_;



    DQ x1_fkm_;
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_x1_fkm_;
    void _callback_x1_fkm_state(const geometry_msgs::msg::PoseStamped& msg);





    //DQ x1d_;
    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_x1d_;





    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::optional<DQ> _try_get_vicon_marker(const std::string& marker_name);
    DQ _geometry_msgs_msg_TransformStamped_to_dq(const geometry_msgs::msg::TransformStamped &msg);

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_agent_3_;


    void _publish_pose_stamped_data(const Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
                                    const DQ& pose);


    void _publish_pose_stamped_data_of_coppeliasim_object_object(const Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
                                                 const std::string& coppeliasim_object_name);



    void _set_joint_states_on_coppeliasim(const std::vector<std::string>& jointnames,
                                          const VectorXd& q);
    void _set_object_pose(const std::string& base_name, const DQ& pose);
    void _connect();

    bool _should_shutdown() const;


    std::tuple<double, double >_get_switching_error(const DQ& x1, const DQ& x2);


public:

    MonitorCoppeliaSim(const MonitorCoppeliaSim&)=delete;
    MonitorCoppeliaSim()=delete;
    ~MonitorCoppeliaSim();

    MonitorCoppeliaSim(std::shared_ptr<Node>& node,
                       const MonitorCoppeliaSimConfiguration &configuration,
                       std::atomic_bool* break_loops);
    void control_loop();



};

}
