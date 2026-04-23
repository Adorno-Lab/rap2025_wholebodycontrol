/*
#    Copyright (c) 2024-2025 Adorno-Lab
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
#include "dqrobotics/interfaces/coppeliasim/robots/UnitreeB1Z1CoppeliaSimZMQRobot.h"
#include "dqrobotics/robots/UnitreeB1Z1MobileRobot.h"
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>

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
    //std::shared_ptr<sas::RobotDriverClient> rdi_;
    std::shared_ptr<UnitreeB1Z1RobotClient> robot_client_;

};

/**
 * @brief B1Z1WholeBodyControl::B1Z1WholeBodyControl ctor of the class
 * @param node
 * @param configuration
 * @param break_loops
 */
B1Z1WholeBodyControl::B1Z1WholeBodyControl(std::shared_ptr<Node> &node,
                                           const ControllerConfiguration &configuration,
                                           std::atomic_bool *break_loops)
    :configuration_{configuration},
    st_break_loops_{break_loops},
    node_{node},
    robot_reached_region_{false},
    T_{configuration.thread_sampling_time_sec},
    clock_{configuration.thread_sampling_time_sec},
    datalogger_client_{node},
    save_data_with_datalogger_{true}
{
    impl_ = std::make_unique<B1Z1WholeBodyControl::Impl>();
    impl_->cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    impl_->qpoases_solver_ = std::make_shared<DQ_QPOASESSolver>();

    impl_->robot_client_ = std::make_shared<UnitreeB1Z1RobotClient>(node_,
                                                                    configuration_.B1_topic_prefix,
                                                                    configuration_.Z1_topic_prefix,
                                                                    UnitreeB1Z1RobotClient::MODE::CONTROL);


    publisher_coppeliasim_frame_x_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix + "/set/coppeliasim_frame_x", 1);


    subscriber_xd_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix + "/get/coppeliasim_frame_xd", 1, std::bind(&B1Z1WholeBodyControl::_callback_xd_state,
                  this, std::placeholders::_1)
        );


    subscriber_gripper_position_from_coppeliasim_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/coppeliasim/get/gripper_position", 1, std::bind(&B1Z1WholeBodyControl::_callback_gripper_position_from_coppeliasim,
                  this, std::placeholders::_1)
        );


}


void B1Z1WholeBodyControl::_spin_and_update(const std::string& msg)
{
    rclcpp::spin_some(node_);
    clock_.update_and_sleep();
    rclcpp::spin_some(node_);
    RCLCPP_INFO_STREAM(node_->get_logger(), msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}


void B1Z1WholeBodyControl::_connect()
{
    impl_->cs_->connect(configuration_.cs_host, configuration_.cs_port, configuration_.cs_TIMEOUT_IN_MILISECONDS);
    z1_jointnames_  = impl_->cs_->get_jointnames_from_object(configuration_.cs_Z1_robotname);
    impl_->robot_cs_ = std::make_shared<UnitreeB1Z1CoppeliaSimZMQRobot>(configuration_.cs_B1_robotname, impl_->cs_);
    rclcpp::spin_some(node_);
}

void B1Z1WholeBodyControl::_update_kinematic_model()
{

    DQ X_IMU;
    DQ X_J1;
    DQ X_J1_OFFSET;
    VectorXd q;
    // wait for the topics
    if (configuration_.debug_wait_for_topics)
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for robot state from ROS2...");
        while (q_arm_.size() == 0 or not is_unit(robot_pose_)){
            rclcpp::spin_some(node_);
            if (_should_shutdown())
                break;
        };
    }else{
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::[DEBUG MODE] No Waiting for topics!");
        robot_pose_ = DQ{1};
        q_arm_ = VectorXd::Zero(6);
        rclcpp::spin_some(node_);
    }

    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Robot state from ROS2 OK!");
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Reading info from CoppeliaSim...");
    const int iter1 = 5;
    for (int i=0; i<iter1;i++)
    {
        try
        {
            X_J1 = impl_->cs_->get_object_pose(z1_jointnames_.at(0));
            X_IMU = impl_->cs_->get_object_pose(configuration_.cs_B1_robotname+"/trunk_respondable");
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
        impl_->cs_->set_object_pose(configuration_.cs_desired_frame, x);
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

/**
 * @brief B1Z1WholeBodyControl::_get_planar_joint_velocities_at_body_frame
 * @param planar_joint_velocities_at_inertial_frame
 * @return
 */
VectorXd B1Z1WholeBodyControl::_get_planar_joint_velocities_at_body_frame(const VectorXd &planar_joint_velocities_at_inertial_frame) const
{
    // Alias
    const VectorXd& ua = planar_joint_velocities_at_inertial_frame; // x_dot, y_dot, phi_dot
    DQ p_dot_a_ab = ua(0)*i_ + ua(1)*j_;
    DQ w_a_ab = ua(2)*k_;
    DQ twist_a = w_a_ab + E_*(p_dot_a_ab + DQ_robotics::cross(robot_pose_.translation(), w_a_ab));
    // Twist_a expressed in the body frame is given as
    DQ twist_b = Ad(robot_pose_.conj(), twist_a);
    VectorXd twist_b_vec = twist_b.vec6(); // [0 0 wb xb_dot yb_dot 0]
                                                 //[xb_dot         yb_dot        wb]
    return DQ_robotics_extensions::CVectorXd({twist_b_vec(3), twist_b_vec(4), twist_b_vec(2)});
}

/**
 * @brief B1Z1WholeBodyControl::control_loop starts the main control loop
 */
void B1Z1WholeBodyControl::control_loop()
{
    try {
        clock_.init();
        rclcpp::spin_some(node_);

        // Wait for the robot drivers
        while (!impl_->robot_client_->is_enabled() && !_should_shutdown())
        {
            _spin_and_update(std::string("Waiting for robot data"));
        }
        _spin_and_update(std::string("Robot is ready!"));

        while (!new_coppeliasim_xd_data_available_ && !_should_shutdown())
        {
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for desired pose from CoppeliaSim ");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }


        _connect();
        _update_kinematic_model();

        for (int i=0;i<100;i++)
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Reading topics: "+std::to_string(i)+"/100");
        }


        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Starting the Robot Constraint Manager...");
        impl_->robot_constraint_manager_ = std::make_shared<DQ_robotics_extensions::RobotConstraintManager>(impl_->cs_,
                                                                                                            impl_->robot_cs_,
                                                                                                            impl_->kin_mobile_manipulator_,
                                                                                                            configuration_.vfi_file,
                                                                                                            true);
        auto  const [q_min, q_max]    = impl_->robot_constraint_manager_->get_configuration_limits();
        auto const [q_dot_min, q_dot_max]= impl_->robot_constraint_manager_->get_configuration_velocity_limits();
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Number of VFI constraints: "+ std::to_string(impl_->robot_constraint_manager_->get_number_of_vfi_constraints()));
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::q_min: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  q_min.transpose());
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::q_max: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  q_max.transpose());

        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::q_dot_min: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  q_dot_min.transpose());
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::q_dot_max: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  q_dot_max.transpose());

        //-----------------------For parking break----------------------------------
        const VectorXd q_break_dot_min = DQ_robotics_extensions::Numpy::vstack(DQ_robotics_extensions::CVectorXd({0,0,0}), -q_dot_max.tail(6));
        const VectorXd q_break_dot_max = DQ_robotics_extensions::Numpy::vstack(DQ_robotics_extensions::CVectorXd({0,0,0}),  q_dot_max.tail(6));

        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::q_break_dot_min: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  q_break_dot_min.transpose());
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::q_break_dot_max: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),   q_break_dot_max.transpose());

        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Starting control loop...");

        VectorXd qi_arm = q_arm_;  // For numerical integration
        VectorXd u;


        DQ_ClassicQPController controller(impl_->kin_mobile_manipulator_, impl_->qpoases_solver_);
        controller.set_control_objective(ControlObjective::Translation);
        controller.set_gain(configuration_.controller_proportional_gain);
        controller.set_damping( configuration_.controller_damping);

        double region_size =  configuration_.controller_target_region_size;
        double region_exit_size =  configuration_.controller_target_exit_size;

        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Controller gain: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  configuration_.controller_proportional_gain);
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Controller damping: ");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),  configuration_.controller_damping);


        if (save_data_with_datalogger_)
        {

            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Waiting for a connection with sas_datalogger...");
            while( (!datalogger_client_.is_enabled()) && (!_should_shutdown()))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                rclcpp::spin_some(node_);
            }
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Connected to sas_datalogger!");
        }else{
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "sas_datalogger is disabled.");
        }

        using DQ_robotics_extensions::Numpy;
        using DQ_robotics_extensions::CVectorXd;

        while(!_should_shutdown())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Running kinematic control loop! ");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            VectorXd q = DQ_robotics_extensions::Numpy::vstack(_get_mobile_platform_configuration_from_pose(robot_pose_), qi_arm);
            DQ x = impl_->robot_model_->fkm(q);
            DQ xd = x;
            _publish_coppeliasim_frame_x(x);



            if (new_coppeliasim_xd_data_available_)
            {
                // There is new data, then we update the desired pose;
                xd = xd_;
            }else{
                // There is no new data about xd, then the desired pose is the same
                // as the current pose. Consequently, the error is zero and robot stops.
                xd = x;
            }

            try {
                //Compute the distance between the end-effector and desired points
                double distance = (x.translation()-xd_.translation()).vec3().norm();

                // If the distance between them is below a threshold and the robot did not reach the target region
                // I stop the robot.
                if (distance < region_size && !robot_reached_region_)
                {
                    //RCLCPP_INFO_STREAM(node_->get_logger(), "::Reached target zone!");
                    u = VectorXd::Zero(9);
                }else
                {
                    // Otherwise, I compute the control inputs to reduce the task error.

                    // If the parking break is eanble
                    if (configuration_.controller_enable_parking_break_when_gripper_is_open)
                    {
                            // if the gripper is open (more than 30 degrees)
                            if (std::abs(target_gripper_position_) > 0.5) // 30 degrees
                            {
                                //When the gripper is open, the target region size and exit size are lower.
                                // This improve the teleoperation accuracy of the arm (Z1 robot).
                                region_size = 0.01;
                                region_exit_size = 0.03;
                                // set the control input constraints to enforce zero velocities in the base (B1 robot).
                                impl_->robot_constraint_manager_->set_configuration_velocity_limits({q_break_dot_min, q_break_dot_max});
                                if (update_handbreak_)
                                {
                                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Parking break enabled!");
                                    update_handbreak_ = false;
                                    update_handbreak_released_ = true;

                                }
                            }else{ // The gripper is closed. No parking break constraints here.
                                region_size =  configuration_.controller_target_region_size;
                                region_exit_size =  configuration_.controller_target_exit_size;
                                impl_->robot_constraint_manager_->set_configuration_velocity_limits({q_dot_min, q_dot_max});
                                if(update_handbreak_released_)
                                {
                                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Parking break disabled!");
                                    update_handbreak_released_ = false;
                                    update_handbreak_ = true;
                                }
                            }
                    }

                    auto [Alim, blim] = impl_->kin_mobile_manipulator_->compute_saturation_constraints(q, {q_dot_min, q_dot_max});
                    impl_->robot_constraint_manager_->add_inequality_constraint(Alim, blim);
                    auto [A,b] = impl_->robot_constraint_manager_->get_inequality_constraints(q, true, false);
                    controller.set_inequality_constraint(A,b);
                    u = controller.compute_setpoint_control_signal(q, xd.translation().vec4());
                }

                if (distance > region_exit_size)
                    robot_reached_region_ = false;

            } catch (const std::exception& e) {
                impl_->robot_client_->set_target_b1_planar_joint_velocities(VectorXd::Zero(3));
                RCLCPP_INFO_STREAM(node_->get_logger(), "::QP not solved!");
                RCLCPP_INFO_STREAM(node_->get_logger(), e.what());
                u = VectorXd::Zero(9);
                break;
            }
            // Numerical integration for to command the arm at joint position level
            qi_arm = qi_arm + T_*u.tail(6);

            ///-----------------Command the robot------------------------------------------------
            // The u.head(3) command velocities for the B1 robot are given with respect to the inertial frame.
            // However, we need to send the velocities with respect to the B1 frame.
            VectorXd ub = DQ_robotics_extensions::get_planar_joint_configuration_velocities_at_body_frame(impl_->robot_client_->get_b1_pose(), u.head(3));
            impl_->robot_client_->set_target_b1_planar_joint_velocities(ub);
            impl_->robot_client_->set_arm_joint_positions(Numpy::vstack(qi_arm, CVectorXd({target_gripper_position_})));



            // Log data
            if (save_data_with_datalogger_)
            {
                datalogger_client_.log("u", u);
                datalogger_client_.log("ub", ub);
                VectorXd vec_x = x.vec8();
                VectorXd vec_xd = xd_.vec8();
                datalogger_client_.log("x", vec_x);
                datalogger_client_.log("xd", vec_xd);
                datalogger_client_.log("q", q);
                datalogger_client_.log("q_dot_min", q_dot_min);
                datalogger_client_.log("q_dot_max", q_dot_max);
                //datalogger_client_.log("q_dot_min_inertial", q_dot_min_inertial);
                //datalogger_client_.log("q_dot_max_inertial", q_dot_max_inertial);
                datalogger_client_.log("robot_reached_region", robot_reached_region_);
                //datalogger_client_.log("J", J);
            }

        }

    }
    catch(const std::exception& e)
    {
        std::cerr<<"::Exception caught::" << e.what() <<std::endl;
    }
}



void B1Z1WholeBodyControl::_callback_gripper_position_from_coppeliasim(const std_msgs::msg::Float64MultiArray &msg)
{
    VectorXd vec_gripper_position = sas::std_vector_double_to_vectorxd(msg.data);
    target_gripper_position_ = vec_gripper_position(0);
}

void B1Z1WholeBodyControl::_callback_xd_state(const geometry_msgs::msg::PoseStamped &msg)
{
    xd_ = sas::geometry_msgs_pose_stamped_to_dq(msg);
    if (!is_unit(xd_))
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "::Problem reading desired pose");
        new_coppeliasim_xd_data_available_ = false;
    }
    new_coppeliasim_xd_data_available_ = true;
}

bool B1Z1WholeBodyControl::_should_shutdown() const
{
    return (*st_break_loops_);
}




void B1Z1WholeBodyControl::_publish_coppeliasim_frame_x(const DQ &pose)
{
    geometry_msgs::msg::PoseStamped ros_msg_frame_x = sas::dq_to_geometry_msgs_pose_stamped(pose);
    publisher_coppeliasim_frame_x_->publish(ros_msg_frame_x);
}


B1Z1WholeBodyControl::~B1Z1WholeBodyControl()
{

}


}
