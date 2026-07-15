/*
#    Copyright (c) 2025 Adorno-Lab
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
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/DQ.h>
#include <sas_core/sas_clock.hpp>


using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{
struct ControlExampleConfiguration
{
    std::string cs_host;
    int cs_port;
    int cs_TIMEOUT_IN_MILISECONDS;
    std::string cs_B1_robotname;
    std::string cs_Z1_robotname;
    std::string B1_topic_prefix;
    std::string Z1_topic_prefix;
    std::string cs_desired_frame;
    double thread_sampling_time_sec;
    std::tuple<VectorXd, VectorXd> configuration_limits;
    std::tuple<VectorXd, VectorXd> configuration_velocity_limits;
    std::string vfi_file;
    double controller_proportional_gain;
    double controller_damping;
};

class ControlExample
{

private:
    ControlExampleConfiguration configuration_;
    std::atomic_bool* st_break_loops_;
    std::shared_ptr<rclcpp::Node> node_;
    bool robot_reached_region_;

    class Impl;
    std::unique_ptr<Impl> impl_;

    bool show_controller_idle_status_;
    bool show_controller_on_status_;

    bool _should_shutdown() const;
    void _update_kinematic_model();

public:
    ControlExample(const ControlExample&)=delete;
    ControlExample()=delete;
    ~ControlExample();

    ControlExample(std::shared_ptr<Node>& node,
                   const ControlExampleConfiguration &configuration,
                   std::atomic_bool* break_loops);


    void control_loop();
};



}
