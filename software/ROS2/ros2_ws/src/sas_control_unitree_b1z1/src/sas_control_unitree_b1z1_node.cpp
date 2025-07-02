
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
//#include <sas_robot_driver_unitree_z1/sas_robot_driver_unitree_z1.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "b1z1_wb_control.hpp"

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
    auto node = std::make_shared<rclcpp::Node>("b1z1_wb_control_node");

    try
    {
        sas::ControllerConfiguration controller_configuration;
        controller_configuration.vfi_file = "/home/clerice/git/unitree-b1-z1/software/ROS2/ros2_ws/src/b1z1_wholebody_control/cfg/vfi_constraints.yaml";

        //sas::get_ros_parameter(node,"vfi_file", controller_configuration.vfi_file);
        //   sas::get_ros_parameter(node,"LIE_DOWN_ROBOT_WHEN_DEINITIALIZE", robot_driver_unitree_b1_configuration.LIE_DOWN_ROBOT_WHEN_DEINITIALIZE);

        auto robot_driver = std::make_shared<sas::B1Z1WholeBodyControl>(node,
                                                                        controller_configuration,
                                                                        &kill_this_process,
                                                                        "sas_b1/sas_B1", "sas_z1/z1_1");

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
