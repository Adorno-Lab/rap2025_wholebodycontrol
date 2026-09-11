#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "control_example/control_example.hpp"

#include <signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

//============================================================
// DEBUG FLAG
//   true  -> use HARDCODED parameters (Qt debugging)
//   false -> read parameters from the launch file
//============================================================
static constexpr bool USE_HARDCODED_PARAMETERS = false;

int main(int argc, char** argv)
{
    if (signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sas_consensus_control_node");

    // The robot configuration is represented by q = [vec8(xbase); q_arm], where xbase is a unit dual quaternion that
    // represents the pose of the quadruped robot.
    // Since no limits are applied for vec8(xbase), we use (-inf, inf).
    std::vector<double> mobile_base_configuration_limits_min = {
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()
    };

    std::vector<double> mobile_base_configuration_limits_max = {
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    };

    try
    {
        sas::ControlExampleConfiguration configuration;

        //---------------------------------------------------------
        //  PARAMETERS
        //---------------------------------------------------------
        if (USE_HARDCODED_PARAMETERS)
        {
            //=====================================================
            // HARDCODED VALUES (copied from launch file)
            //=====================================================
            configuration.cs_host                    = "localhost";
            configuration.cs_port                    = 23000;
            configuration.cs_TIMEOUT_IN_MILISECONDS  = 5000;
            configuration.cs_B1_robotname            = "UnitreeB1_1";
            configuration.cs_Z1_robotname            = "UnitreeZ1";
            configuration.cs_desired_frame           = "xd1";
            configuration.B1_topic_prefix            = "sas_b1/b1_1";
            configuration.Z1_topic_prefix            = "sas_z1/z1_1";
            configuration.thread_sampling_time_sec   = 0.002;
            configuration.controller_proportional_gain = 8.0;
            configuration.controller_damping         = 0.1;
            configuration.controller_target_region_size = 0.15;
            configuration.controller_target_exit_size   = 0.2;
            configuration.vfi_file                   = "/home/juanjqo/git/rap2025_wholebodycontrol/software/ROS2/ros2_ws/src/control_example/cfg/vfi_config.yaml";

            RCLCPP_INFO_STREAM(node->get_logger(), "::Using HARDCODED parameters (debug mode).");
        }
        else
        {
            //=====================================================
            // PARAMETERS FROM LAUNCH FILE
            //=====================================================
            sas::get_ros_parameter(node, "cs_host", configuration.cs_host);
            sas::get_ros_parameter(node, "cs_port", configuration.cs_port);
            sas::get_ros_parameter(node, "cs_TIMEOUT_IN_MILISECONDS", configuration.cs_TIMEOUT_IN_MILISECONDS);
            sas::get_ros_parameter(node, "cs_B1_robotname", configuration.cs_B1_robotname);
            sas::get_ros_parameter(node, "cs_Z1_robotname", configuration.cs_Z1_robotname);
            sas::get_ros_parameter(node, "B1_topic_prefix", configuration.B1_topic_prefix);
            sas::get_ros_parameter(node, "Z1_topic_prefix", configuration.Z1_topic_prefix);
            sas::get_ros_parameter(node, "cs_desired_frame", configuration.cs_desired_frame);
            sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.thread_sampling_time_sec);
            sas::get_ros_parameter(node, "controller_proportional_gain", configuration.controller_proportional_gain);
            sas::get_ros_parameter(node, "controller_damping", configuration.controller_damping);
            sas::get_ros_parameter(node, "vfi_file", configuration.vfi_file);
            sas::get_ros_parameter(node, "controller_target_region_size", configuration.controller_target_region_size);
            sas::get_ros_parameter(node, "controller_target_exit_size", configuration.controller_target_exit_size);

            RCLCPP_INFO_STREAM(node->get_logger(), "::Using parameters from LAUNCH FILE.");
        }

        ///------------------------------Configuration Limits-----------------------------------------------
        VectorXd q_base_min = sas::std_vector_double_to_vectorxd(mobile_base_configuration_limits_min);
        VectorXd q_base_max = sas::std_vector_double_to_vectorxd(mobile_base_configuration_limits_max);

        std::vector<double> arm_configuration_limits_min;
        std::vector<double> arm_configuration_limits_max;
        std::vector<double> arm_configuration_buffer;

        if (USE_HARDCODED_PARAMETERS)
        {
            //=====================================================
            // HARDCODED ARM LIMITS (degrees, from launch file)
            //=====================================================
            arm_configuration_limits_min = {-30.0,  45.0, -90.0, -80.0, -45.0, -160.0};
            arm_configuration_limits_max = { 30.0, 165.0,   0.0,  80.0,  45.0,  160.0};
            arm_configuration_buffer     = {  5.0,   5.0,   5.0,   5.0,   5.0,    5.0};
        }
        else
        {
            sas::get_ros_parameter(node, "arm_configuration_limits_min", arm_configuration_limits_min);
            sas::get_ros_parameter(node, "arm_configuration_limits_max", arm_configuration_limits_max);
            sas::get_ros_parameter(node, "arm_configuration_buffer", arm_configuration_buffer);
        }

        VectorXd q_arm_min = deg2rad(sas::std_vector_double_to_vectorxd(arm_configuration_limits_min));
        VectorXd q_arm_max = deg2rad(sas::std_vector_double_to_vectorxd(arm_configuration_limits_max));

        VectorXd q_min(q_base_min.size() + q_arm_min.size());
        q_min << q_base_min, q_arm_min;

        VectorXd q_max(q_base_max.size() + q_arm_max.size());
        q_max << q_base_max, q_arm_max;

        configuration.configuration_limits = {q_min, q_max};

        ///------------------------ Arm Configuration buffer------------------
        VectorXd b_arm_buffer = deg2rad(sas::std_vector_double_to_vectorxd(arm_configuration_buffer));
        configuration.b_arm_buffer = b_arm_buffer;

        ///------------------------------Configuration Velocity Limits----------------------------------------
        std::vector<double> mobile_base_configuration_velocity_limits_min;
        std::vector<double> mobile_base_configuration_velocity_limits_max;
        std::vector<double> arm_configuration_velocity_limits_min;
        std::vector<double> arm_configuration_velocity_limits_max;

        if (USE_HARDCODED_PARAMETERS)
        {
            //=====================================================
            // HARDCODED VELOCITY LIMITS (from launch file)
            //=====================================================
            mobile_base_configuration_velocity_limits_min = {-0.2, -0.2, -0.2, -0.2, -0.2, -0.2};
            mobile_base_configuration_velocity_limits_max = { 0.2,  0.2,  0.2,  0.3,  0.2,  0.2};
            arm_configuration_velocity_limits_min         = {-1.57, -1.57, -1.57, -1.57, -1.57, -1.57}; // rad/s
            arm_configuration_velocity_limits_max         = { 1.57,  1.57,  1.57,  1.57,  1.57,  1.57}; // rad/s
        }
        else
        {
            sas::get_ros_parameter(node, "mobile_base_configuration_velocity_limits_min", mobile_base_configuration_velocity_limits_min);
            sas::get_ros_parameter(node, "mobile_base_configuration_velocity_limits_max", mobile_base_configuration_velocity_limits_max);
            sas::get_ros_parameter(node, "arm_configuration_velocity_limits_min", arm_configuration_velocity_limits_min);
            sas::get_ros_parameter(node, "arm_configuration_velocity_limits_max", arm_configuration_velocity_limits_max);
        }

        VectorXd q_dot_min_base = sas::std_vector_double_to_vectorxd(mobile_base_configuration_velocity_limits_min);
        VectorXd q_dot_min_arm  = sas::std_vector_double_to_vectorxd(arm_configuration_velocity_limits_min);
        VectorXd q_dot_min(q_dot_min_base.size() + q_dot_min_arm.size());
        q_dot_min << q_dot_min_base, q_dot_min_arm;

        VectorXd q_dot_max_base = sas::std_vector_double_to_vectorxd(mobile_base_configuration_velocity_limits_max);
        VectorXd q_dot_max_arm  = sas::std_vector_double_to_vectorxd(arm_configuration_velocity_limits_max);
        VectorXd q_dot_max(q_dot_max_base.size() + q_dot_max_arm.size());
        q_dot_max << q_dot_max_base, q_dot_max_arm;

        configuration.configuration_velocity_limits = {q_dot_min, q_dot_max};
        ///-----------------------------------------------------------------------------------------------

        auto control = std::make_shared<sas::ControlExample>(node, configuration, &kill_this_process);
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
