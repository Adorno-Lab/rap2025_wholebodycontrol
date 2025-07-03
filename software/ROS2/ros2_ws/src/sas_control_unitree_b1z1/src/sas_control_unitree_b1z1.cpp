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
#include "sas_control_unitree_b1z1.hpp"
#include "UnitreeB1Z1CoppeliaSimZMQRobot.hpp"
#include "UnitreeB1Z1MobileRobot.hpp"
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include <sas_core/eigen3_std_conversions.hpp>

namespace sas
{

class B1Z1WholeBodyControl::Impl
{
public:
    std::shared_ptr<DQ_QPOASESSolver> qpoases_solver_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::shared_ptr<DQ_robotics_extensions::RobotConstraintManager> robot_constraint_manager_;
    std::shared_ptr<DQ_Kinematics> robot_model_;
    std::shared_ptr<UnitreeB1Z1MobileRobot> kin_mobile_manipulator_;
    std::shared_ptr<UnitreeB1Z1CoppeliaSimZMQRobot> robot_cs_;
};


B1Z1WholeBodyControl::B1Z1WholeBodyControl(std::shared_ptr<Node> &node,
                                           const ControllerConfiguration &configuration,
                                           std::atomic_bool *break_loops)
    :cs_host_{configuration.cs_host},
    cs_port_{configuration.cs_port},
    cs_TIMEOUT_IN_MILISECONDS_{configuration.cs_TIMEOUT_IN_MILISECONDS},
    cs_Z1_robotname_{configuration.cs_Z1_robotname},
    cs_B1_robotname_{configuration.cs_B1_robotname},
    st_break_loops_{break_loops},
    topic_prefix_b1_{configuration.B1_topic_prefix},
    topic_prefix_z1_{configuration.Z1_topic_prefix},
    node_{node},
    vfi_file_{configuration.vfi_file},
    T_{configuration.thread_sampling_time_sec},
    print_count_{0},
    clock_{configuration.thread_sampling_time_sec},
    datalogger_client_{node}, //node, 10
    save_data_with_datalogger_{false}
{
    impl_ = std::make_unique<B1Z1WholeBodyControl::Impl>();
    impl_->cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    impl_->qpoases_solver_ = std::make_shared<DQ_QPOASESSolver>();


    publisher_target_arm_positions_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        topic_prefix_z1_ + "/set/target_joint_positions", 1);

    publisher_target_holonomic_velocities_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        topic_prefix_b1_ + "/set/holonomic_target_velocities", 1);

    subscriber_pose_state_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/coppeliasim/get/robot_pose", 1, std::bind(&B1Z1WholeBodyControl::_callback_pose_state,
                  this, std::placeholders::_1)
        );
    subscriber_Z1_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_prefix_z1_ + "/get/joint_states", 1, std::bind(&B1Z1WholeBodyControl::_callback_Z1_joint_states,
                  this, std::placeholders::_1)
        );

    publisher_coppeliasim_frame_x_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_prefix_b1_ + "/set/coppeliasim_frame_x", 1);


    subscriber_xd_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_prefix_b1_ + "/get/coppeliasim_frame_xd", 1, std::bind(&B1Z1WholeBodyControl::_callback_xd_state,
                  this, std::placeholders::_1)
        );


    subscriber_gripper_position_from_coppeliasim_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/coppeliasim/get/gripper_position", 1, std::bind(&B1Z1WholeBodyControl::_callback_gripper_position_from_coppeliasim,
                  this, std::placeholders::_1)
        );


}





void B1Z1WholeBodyControl::_connect()
{
    impl_->cs_->connect(cs_host_, cs_port_, cs_TIMEOUT_IN_MILISECONDS_);
    z1_jointnames_  = impl_->cs_->get_jointnames_from_object(cs_Z1_robotname_);
    impl_->robot_cs_ = std::make_shared<UnitreeB1Z1CoppeliaSimZMQRobot>(cs_B1_robotname_, impl_->cs_);
    rclcpp::spin_some(node_);
}

void B1Z1WholeBodyControl::_update_kinematic_model()
{

    DQ X_IMU;
    DQ X_J1;
    DQ X_J1_OFFSET;
    VectorXd q;
    // wait for the topics
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Reading robot state from ROS2...");
    while (q_arm_.size() == 0 or not is_unit(robot_pose_)){
        rclcpp::spin_some(node_);
        if (_should_shutdown())
            break;
    };
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Robot state from ROS2 OK!");
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Reading info from CoppeliaSim...");
    const int iter1 = 5;
    for (int i=0; i<iter1;i++)
    {
        try
        {
            X_J1 = impl_->cs_->get_object_pose(z1_jointnames_.at(0));
            X_IMU = impl_->cs_->get_object_pose(cs_B1_robotname_+"/trunk_respondable");
            q = DQ_robotics_extensions::Numpy::vstack(_get_mobile_platform_configuration_from_pose(robot_pose_), q_arm_);
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Reading info from CoppeliaSim: "+std::to_string(i)+"/"+std::to_string(iter1));
            X_J1_OFFSET = X_IMU.conj()*X_J1;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), std::string("::Exception::") + e.what());
            std::cerr << std::string("::Exception::") << e.what();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (_should_shutdown())
            break;
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Updating kinematic model...");
        /*********************************************
         * ROBOT KINEMATIC MODEL
         * *******************************************/
        impl_->kin_mobile_manipulator_ = std::make_shared<UnitreeB1Z1MobileRobot>();
        impl_->kin_mobile_manipulator_->update_base_offset(X_J1_OFFSET);
        impl_->kin_mobile_manipulator_->update_base_height_from_IMU(X_IMU);
        impl_->robot_model_ = std::shared_ptr<DQ_Kinematics>(impl_->kin_mobile_manipulator_);
        DQ x = impl_->robot_model_->fkm(q);
        impl_->cs_->set_object_pose("xd", x);
    }

}

VectorXd B1Z1WholeBodyControl::_get_mobile_platform_configuration_from_pose(const DQ &pose) const
{
    DQ x = pose;
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}


void B1Z1WholeBodyControl::control_loop()
{
    try {
        clock_.init();
        rclcpp::spin_some(node_);
        _connect();
        _update_kinematic_model();

        for (int i=0;i<100;i++)
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Reading topics: "+std::to_string(i)+"/100");
        }


        VectorXd qarm_target = q_arm_;
        qarm_target(5) = -M_PI/2;
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Setting custom arm configuration...");

        const int size = 200;
        auto qarm_inter = DQ_robotics_extensions::Numpy::linspace(q_arm_, qarm_target, size);
        for (int i=0;i<200;i++)
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            _publish_target_Z1_commands(qarm_inter.col(i), target_gripper_position_);
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Setting custom arm configuration: "+std::to_string(i)+"/100");
        }
        _publish_target_Z1_commands(qarm_target, target_gripper_position_);
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::custom arm configuration set!");


        impl_->robot_constraint_manager_ = std::make_shared<DQ_robotics_extensions::RobotConstraintManager>(impl_->cs_,
                                                                                                            impl_->robot_cs_,
                                                                                                            impl_->kin_mobile_manipulator_,
                                                                                                            vfi_file_, true);
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Number of VFI constraints: "+ std::to_string(impl_->robot_constraint_manager_->get_number_of_vfi_constraints()));

    }
    catch(const std::exception& e)
    {
        std::cerr<<"::Exception caught::" << e.what() <<std::endl;
    }
}

void B1Z1WholeBodyControl::_callback_Z1_joint_states(const sensor_msgs::msg::JointState &msg)
{
    VectorXd qarm = sas::std_vector_double_to_vectorxd(msg.position);
    q_arm_ = qarm.head(6);
}

void B1Z1WholeBodyControl::_callback_pose_state(const geometry_msgs::msg::PoseStamped &msg)
{
    robot_pose_ =   sas::geometry_msgs_pose_stamped_to_dq(msg);
}

void B1Z1WholeBodyControl::_callback_gripper_position_from_coppeliasim(const std_msgs::msg::Float64MultiArray &msg)
{
    VectorXd vec_gripper_position = sas::std_vector_double_to_vectorxd(msg.data);
    target_gripper_position_ = vec_gripper_position(0);
}

void B1Z1WholeBodyControl::_callback_xd_state(const geometry_msgs::msg::PoseStamped &msg)
{
    xd_ = sas::geometry_msgs_pose_stamped_to_dq(msg);
}

bool B1Z1WholeBodyControl::_should_shutdown() const
{
    return (*st_break_loops_);
}

void B1Z1WholeBodyControl::_publish_target_B1_commands(const VectorXd &u_base_vel)
{
    VectorXd u_base = u_base_vel;
    // The B1 robot does not move with velocities below 0.03, but the legs continue to move.
    // Therefore, we enforce a deadband.
    const double min_vel = 0.032;
    for (int i=0;i<u_base.size();i++)
    {
        if (std::abs(u_base(i))<=min_vel)
            u_base(i) = 0.0;
    }
    std_msgs::msg::Float64MultiArray ros_msg_u_base;
    ros_msg_u_base.data = sas::vectorxd_to_std_vector_double(u_base_vel);
    publisher_target_holonomic_velocities_->publish(ros_msg_u_base);
}

void B1Z1WholeBodyControl::_publish_target_Z1_commands(const VectorXd &u_arm_positions, const double &gripper_position)
{
    std_msgs::msg::Float64MultiArray ros_msg_u_arm_commands;
    VectorXd commands = DQ_robotics_extensions::Numpy::vstack(u_arm_positions,
                                                              DQ_robotics_extensions::CVectorXd({gripper_position}));
    ros_msg_u_arm_commands.data = sas::vectorxd_to_std_vector_double(commands);
    publisher_target_arm_positions_->publish(ros_msg_u_arm_commands);
}

void B1Z1WholeBodyControl::_publish_coppeliasim_frame_x(const DQ &pose)
{
    geometry_msgs::msg::PoseStamped ros_msg_frame_x;
    VectorXd position = pose.translation().vec3();
    ros_msg_frame_x.pose.position.x = position(0);
    ros_msg_frame_x.pose.position.y = position(1);
    ros_msg_frame_x.pose.position.z = position(2);

    VectorXd orientation = pose.rotation().vec4();
    ros_msg_frame_x.pose.orientation.w = orientation(0);
    ros_msg_frame_x.pose.orientation.x = orientation(1);
    ros_msg_frame_x.pose.orientation.y = orientation(2);
    ros_msg_frame_x.pose.orientation.z = orientation(3);

    publisher_coppeliasim_frame_x_->publish(ros_msg_frame_x);
}


B1Z1WholeBodyControl::~B1Z1WholeBodyControl()
{

}


}
