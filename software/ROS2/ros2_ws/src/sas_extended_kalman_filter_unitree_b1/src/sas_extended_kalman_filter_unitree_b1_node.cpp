
#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_extended_kalman_filter_unitree_b1/sas_extended_kalman_filter_unitree_b1.hpp>
#include <dqrobotics/utils/DQ_Math.h>


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
        throw std::runtime_error("::Error setting the signal int handler.");

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("extended_kalman_filter_dev");


    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");
        sas::ExtendedKalmanFilterConfiguration configuration;
        sas::get_ros_parameter(node,"thread_sampling_time_sec",configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node,"topic_prefix",configuration.topic_prefix);
        sas::get_ros_parameter(node, "robot_vicon_marker", configuration.robot_vicon_marker);
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");

        auto ekf_driver = std::make_shared<sas::ExtendedKalmanFilter>(node, configuration, &kill_this_process);
        ekf_driver->control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    return 0;
}
