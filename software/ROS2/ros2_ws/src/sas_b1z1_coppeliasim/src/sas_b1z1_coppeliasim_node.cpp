
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

        sas::get_ros_parameter(node,"host",configuration.host);
        sas::get_ros_parameter(node,"port",configuration.port);
        sas::get_ros_parameter(node,"TIMEOUT_IN_MILISECONDS",configuration.TIMEOUT_IN_MILISECONDS);

        auto robot_driver = std::make_shared<sas::B1Z1CoppeliaSimROS>(node,
                                                                      configuration,
                                                                      &kill_this_process,
                                                                         "sas_b1/sas_B1",
                                                                         "sas_z1/z1_1");

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

