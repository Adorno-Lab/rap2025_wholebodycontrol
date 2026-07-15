#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/utils/DQ_Math.h>
//#include "sas_consensus_control_unitree_b1z1/sas_consensus_control_unitree_b1z1.hpp"
#include "control_example/control_example.hpp"

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
    auto node = std::make_shared<rclcpp::Node>("sas_consensus_control_node");

    try
    {

        sas::ControlExampleConfiguration configuration;
        sas::get_ros_parameter(node,"cs_host",configuration.cs_host);
        sas::get_ros_parameter(node,"cs_port",configuration.cs_port);
        sas::get_ros_parameter(node,"cs_TIMEOUT_IN_MILISECONDS",configuration.cs_TIMEOUT_IN_MILISECONDS);
        sas::get_ros_parameter(node,"cs_B1_robotname",configuration.cs_B1_robotname);
        sas::get_ros_parameter(node,"cs_Z1_robotname",configuration.cs_Z1_robotname);
        sas::get_ros_parameter(node,"B1_topic_prefix",configuration.B1_topic_prefix);
        sas::get_ros_parameter(node,"Z1_topic_prefix",configuration.Z1_topic_prefix);
        sas::get_ros_parameter(node, "cs_desired_frame", configuration.cs_desired_frame);
        sas::get_ros_parameter(node,"thread_sampling_time_sec",configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node,"controller_proportional_gain",configuration.controller_proportional_gain);
        sas::get_ros_parameter(node,"controller_damping",configuration.controller_damping);
        sas::get_ros_parameter(node,"vfi_file", configuration.vfi_file);


        ///------------------------------Configuration Limits-----------------------------------------------
        std::vector<double> mobile_base_configuration_limits_min;
        std::vector<double> mobile_base_configuration_limits_max;
        sas::get_ros_parameter(node,"mobile_base_configuration_limits_min", mobile_base_configuration_limits_min);
        sas::get_ros_parameter(node,"mobile_base_configuration_limits_max", mobile_base_configuration_limits_max);

        VectorXd q_base_min = sas::std_vector_double_to_vectorxd(mobile_base_configuration_limits_min);
        VectorXd q_base_max = sas::std_vector_double_to_vectorxd(mobile_base_configuration_limits_max);


        std::vector<double> arm_configuration_limits_min;
        std::vector<double> arm_configuration_limits_max;
        sas::get_ros_parameter(node,"arm_configuration_limits_min", arm_configuration_limits_min);
        sas::get_ros_parameter(node,"arm_configuration_limits_max", arm_configuration_limits_max);

        VectorXd q_arm_min =  deg2rad(sas::std_vector_double_to_vectorxd(arm_configuration_limits_min));
        VectorXd q_arm_max =  deg2rad(sas::std_vector_double_to_vectorxd(arm_configuration_limits_max));

        VectorXd q_min(q_base_min.size() + q_arm_min.size());
        q_min << q_base_min, q_arm_min;

        VectorXd q_max(q_base_max.size() + q_arm_max.size());
        q_max << q_base_max, q_arm_max;

        configuration.configuration_limits = {q_min, q_max};

        ///------------------------------Configuration Velocity Limits----------------------------------------

        std::vector<double> mobile_base_configuration_velocity_limits_min;
        std::vector<double> mobile_base_configuration_velocity_limits_max;
        sas::get_ros_parameter(node,"mobile_base_configuration_velocity_limits_min", mobile_base_configuration_velocity_limits_min);
        sas::get_ros_parameter(node,"mobile_base_configuration_velocity_limits_max", mobile_base_configuration_velocity_limits_max);


        std::vector<double> arm_configuration_velocity_limits_min;
        std::vector<double> arm_configuration_velocity_limits_max;

        sas::get_ros_parameter(node,"arm_configuration_velocity_limits_min",  arm_configuration_velocity_limits_min);
        sas::get_ros_parameter(node,"arm_configuration_velocity_limits_max", arm_configuration_velocity_limits_max);

        VectorXd q_dot_min_base = sas::std_vector_double_to_vectorxd(mobile_base_configuration_velocity_limits_min);
        VectorXd q_dot_min_arm = sas::std_vector_double_to_vectorxd(arm_configuration_velocity_limits_min);
        VectorXd q_dot_min(q_dot_min_base.size() + q_dot_min_arm.size());
        q_dot_min << q_dot_min_base, q_dot_min_arm;

        VectorXd q_dot_max_base = sas::std_vector_double_to_vectorxd(mobile_base_configuration_velocity_limits_max);
        VectorXd q_dot_max_arm = sas::std_vector_double_to_vectorxd(arm_configuration_velocity_limits_max);
        VectorXd q_dot_max(q_dot_max_base.size() + q_dot_max_arm.size());
        q_dot_max << q_dot_max_base, q_dot_max_arm;

        configuration.configuration_velocity_limits = {q_dot_min, q_dot_max};
        ///-----------------------------------------------------------------------------------------------





        auto control = std::make_shared<sas::ControlExample>(node,configuration, &kill_this_process);
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");
        control->control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;


}
