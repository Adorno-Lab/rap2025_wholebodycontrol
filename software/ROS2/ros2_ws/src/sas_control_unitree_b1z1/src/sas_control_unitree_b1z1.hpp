#pragma once
#include <atomic>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <sas_datalogger/sas_datalogger_client.hpp>


//using namespace Eigen;

using namespace rclcpp;

namespace sas
{

struct ControllerConfiguration
{
    std::string vfi_file;
};

class B1Z1WholeBodyControl
{
protected:
    std::atomic_bool* st_break_loops_;
    std::string topic_prefix_b1_;
    std::string topic_prefix_z1_;

    std::shared_ptr<rclcpp::Node> node_;
    DQ robot_pose_; // pose of the B1 robot;
    VectorXd q_arm_;
    double target_gripper_position_{0};
    std::vector<std::string> z1_jointnames_;

    std::string vfi_file_;

private:
    MatrixXd I_;

    VectorXd qmin_arm;
    VectorXd qmax_arm;
    VectorXd q_dot_arm_max;

    VectorXd qmin;
    VectorXd qmax;

    VectorXd q_dot_min;
    VectorXd q_dot_max;


    VectorXd q_break_dot_min;
    VectorXd q_break_dot_max;


    bool update_handbreak_{true};
    bool update_handbreak_released_{true};

    double controller_proportional_gain_{8};
    double controller_damping_{0.05};

    double timer_period_;
    double T_;

    int print_count_;

    sas::Clock clock_;
    sas::DataloggerClient datalogger_client_;
    bool save_data_with_datalogger_;


    //also equivalent to rclcpp::TimerBase::SharedPtr
    std::shared_ptr<rclcpp::TimerBase> timer_;


    Subscription <sensor_msgs::msg::JointState>::SharedPtr subscriber_Z1_joint_states_;
    void _callback_Z1_joint_states(const sensor_msgs::msg::JointState& msg);

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_state_;
    void _callback_pose_state(const geometry_msgs::msg::PoseStamped& msg);

    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_holonomic_velocities_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_arm_positions_;


    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_gripper_position_from_coppeliasim_;
    void _callback_gripper_position_from_coppeliasim(const std_msgs::msg::Float64MultiArray& msg);


    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_frame_x_;


    DQ xd_;
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_xd_;
    void _callback_xd_state(const geometry_msgs::msg::PoseStamped& msg);




    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;

protected:

    bool _should_shutdown() const;

    void _publish_target_B1_commands(const VectorXd& u_base_vel);
    void _publish_target_Z1_commands(const VectorXd& u_arm_positions, const double& gripper_position);
    void _publish_coppeliasim_frame_x(const DQ& pose);
    void _connect();
    void _update_kinematic_model();

    VectorXd _get_mobile_platform_configuration_from_pose(const DQ& pose) const;

public:

    B1Z1WholeBodyControl(const B1Z1WholeBodyControl&)=delete;
    B1Z1WholeBodyControl()=delete;
    ~B1Z1WholeBodyControl();

    B1Z1WholeBodyControl(std::shared_ptr<Node>& node,
                         const  ControllerConfiguration &configuration,
                         std::atomic_bool* break_loops,
                         const std::string& topic_prefix_b1,
                         const std::string& topic_prefix_z1);


    void control_loop();

};



}
