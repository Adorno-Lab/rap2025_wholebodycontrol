
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "sas_b1z1_coppeliasim_ros.hpp"

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
    auto node = std::make_shared<rclcpp::Node>("sas_b1z1_coppeliasim_node");

    try
    {
        sas::RobotDriverB1Z1CoppeliaSimConfiguration configuration;

        sas::get_ros_parameter(node,"cs_host",configuration.cs_host);
        sas::get_ros_parameter(node,"cs_port",configuration.cs_port);
        sas::get_ros_parameter(node,"cs_TIMEOUT_IN_MILISECONDS",configuration.cs_TIMEOUT_IN_MILISECONDS);
        sas::get_ros_parameter(node,"cs_B1_robotname",configuration.cs_B1_robotname);
        sas::get_ros_parameter(node,"cs_Z1_robotname",configuration.cs_Z1_robotname);
        sas::get_ros_parameter(node,"B1_topic_prefix",configuration.B1_topic_prefix);
        sas::get_ros_parameter(node,"Z1_topic_prefix",configuration.Z1_topic_prefix);
        sas::get_ros_parameter(node,"thread_sampling_time_sec",configuration.thread_sampling_time_sec);

        auto robot_driver = std::make_shared<sas::B1Z1CoppeliaSimROS>(node,
                                                                      configuration,
                                                                      &kill_this_process);

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

