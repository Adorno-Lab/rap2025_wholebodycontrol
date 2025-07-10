/*
#    Copyright (c) 2024 Adorno-Lab
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

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "sas_control_unitree_b1z1.hpp"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{

    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("sas_control_unitree_b1z1_node");

    try
    {
        sas::ControllerConfiguration configuration;


        sas::get_ros_parameter(node,"cs_host",configuration.cs_host);
        sas::get_ros_parameter(node,"cs_port",configuration.cs_port);
        sas::get_ros_parameter(node,"cs_TIMEOUT_IN_MILISECONDS",configuration.cs_TIMEOUT_IN_MILISECONDS);
        sas::get_ros_parameter(node,"cs_B1_robotname",configuration.cs_B1_robotname);
        sas::get_ros_parameter(node,"cs_Z1_robotname",configuration.cs_Z1_robotname);
        sas::get_ros_parameter(node,"vfi_file", configuration.vfi_file);
        sas::get_ros_parameter(node,"B1_topic_prefix",configuration.B1_topic_prefix);
        sas::get_ros_parameter(node,"Z1_topic_prefix",configuration.Z1_topic_prefix);
        sas::get_ros_parameter(node,"thread_sampling_time_sec",configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node,"controller_proportional_gain",configuration.controller_proportional_gain);
        sas::get_ros_parameter(node,"controller_damping",configuration.controller_damping);
        sas::get_ros_parameter(node,"debug_wait_for_topics", configuration.debug_wait_for_topics);
        sas::get_ros_parameter(node,"controller_target_region_size", configuration.controller_target_region_size);
        sas::get_ros_parameter(node,"controller_target_exit_size", configuration.controller_target_exit_size);
        sas::get_ros_parameter(node, "controller_enable_parking_break_when_gripper_is_open", configuration.controller_enable_parking_break_when_gripper_is_open);

        auto robot_driver = std::make_shared<sas::B1Z1WholeBodyControl>(node,
                                                                        configuration,
                                                                        &kill_this_process); //,"sas_b1/b1_1", "sas_z1/z1_1");

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        robot_driver->control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    sas::display_signal_handler_none_bug_info(node);
    return 0;


}
