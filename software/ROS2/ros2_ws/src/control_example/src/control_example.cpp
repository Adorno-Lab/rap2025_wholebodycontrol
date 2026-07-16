/*
#    Copyright (c) 2025-2026 Adorno-Lab
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
#   This example is based on https://github.com/MarinhoLab/sas_robot_driver_ur
#
# ################################################################
*/

#include "control_example/control_example.hpp"
#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>
#include <sas_core/sas_clock.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>
#include "dqrobotics/interfaces/coppeliasim/robots/UnitreeB1Z1CoppeliaSimZMQRobot.h"
//#include "dqrobotics/robots/UnitreeB1Z1MobileRobot.h"
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics_extensions/robot_constraint_editor/vfi_configuration_file_yaml.hpp>
#include <sas_datalogger/sas_datalogger_client.hpp>

#include <dqrobotics/robots/CFFSerialRobot.h>
#include <dqrobotics/robots/UnitreeZ1Robot.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

VectorXd _get_rotation_error(const DQ &x, const DQ &xd)
{
    VectorXd error_1 =  vec4( x.rotation().conj()*xd.rotation() - 1 );
    VectorXd error_2 =  vec4( x.rotation().conj()*xd.rotation() + 1 );

    double norm_1 = error_1.norm();
    double norm_2 = error_2.norm();

    if (norm_1 < norm_2){
        return error_1;
    }else{
        return error_2;
    }
}

namespace sas
{

class ControlExample::Impl
{
public:

    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::shared_ptr<DQ_Kinematics> robot_model_;
    std::shared_ptr<CFFSerialRobot> kin_mobile_manipulator_;
    std::shared_ptr<UnitreeB1Z1CoppeliaSimZMQRobot> robot_cs_;
    std::shared_ptr<UnitreeB1Z1RobotClient> robot_client_;
    std::shared_ptr<rclcpp::Node> node__;

    Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_coppeliasim_frame_x_;


    DQ xd_;
    bool new_coppeliasim_xd_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_xd_;

    MatrixXd Aeq_wm_;
    VectorXd beq_wm_;

    int dim_control_inputs_;



    std::vector<std::string> z1_jointnames_;
    sas::Clock clock_;
    sas::DataloggerClient datalogger_client_;
    Impl(std::shared_ptr<Node>& node, const double& thread_sampling_time_sec,
         std::string& B1_topic_prefix):
        node__{node}, clock_{thread_sampling_time_sec}, datalogger_client_{node}
    {
        publisher_coppeliasim_frame_x_ = node__->create_publisher<geometry_msgs::msg::PoseStamped>(
            B1_topic_prefix + "/set/coppeliasim_frame_x", 1);

        subscriber_xd_ = node__->create_subscription<geometry_msgs::msg::PoseStamped>(
            B1_topic_prefix + "/get/coppeliasim_frame_xd", 1, std::bind(&Impl::callback_xd_state,
                      this, std::placeholders::_1)
            );



    }

    /**
     * @brief spin_and_update
     * @param msg
     */
    void spin_and_update(const std::string& msg)
    {
        rclcpp::spin_some(node__);
        clock_.update_and_sleep();
        rclcpp::spin_some(node__);
        RCLCPP_INFO_STREAM(node__->get_logger(), msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    /**
     * @brief connect performs a connection to the simulation scene in CoppeliaSim
     * @param host
     * @param port
     * @param TIMEOUT_IN_MILISECONDS
     * @param B1_robotname
     * @param Z1_robotname
     */
    void connect(const std::string& host, const int& port, const int&TIMEOUT_IN_MILISECONDS,
                 const std::string& B1_robotname, const std::string& Z1_robotname)
    {
        cs_->connect(host, port, TIMEOUT_IN_MILISECONDS);
        z1_jointnames_  = cs_->get_jointnames_from_object(Z1_robotname);
        robot_cs_ = std::make_shared<UnitreeB1Z1CoppeliaSimZMQRobot>(B1_robotname+"/trunk_respondable",
                                                                     cs_,
                                                                     UnitreeB1Z1CoppeliaSimZMQRobot::MODEL::CFF_MANIPULATOR);
        rclcpp::spin_some(node__);
    }


    /**
     * @brief publish_coppeliasim_frame_x publishes the pose on the coppeliasim topic
     * @param pose
     */
    void publish_coppeliasim_frame_x(const DQ& pose)
    {
        geometry_msgs::msg::PoseStamped msg = sas::dq_to_geometry_msgs_pose_stamped(pose);
        publisher_coppeliasim_frame_x_->publish(msg);
    }

    void callback_xd_state(const geometry_msgs::msg::PoseStamped& msg)
    {
        xd_ = sas::geometry_msgs_pose_stamped_to_dq(msg);
        if (!is_unit(xd_))
        {
            RCLCPP_INFO_STREAM(node__->get_logger(), "::Problem reading desired pose");
            new_coppeliasim_xd_data_available_ = false;
        }
        new_coppeliasim_xd_data_available_ = true;
    }

    /**
     * @brief get_desired_pose gets the desired pose
     * @return The desired pose
     */
    DQ get_desired_pose()
    {
        return xd_;
    }

    /**
     * @brief new_coppeliasim_xd_data_available checks if new data about the desired pose is available
     * @return True if new data is available. False otherwise.
     */
    bool new_coppeliasim_xd_data_available()
    {
        return new_coppeliasim_xd_data_available_;
    }

};

bool ControlExample::_should_shutdown() const
{
    return (*st_break_loops_);
}

/**
 * @brief ControlExample::_update_kinematic_model
 */
void ControlExample::_update_kinematic_model()
{
    DQ offset;
    VectorXd q;
    // wait for the topics

    while (!impl_->robot_client_->is_arm_data_available() ||  !is_unit(impl_->robot_client_->get_b1_pose()))
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for robot state from ROS2...");
        rclcpp::spin_some(node_);
        if (_should_shutdown())
            break;
    };

    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Robot state from ROS2 OK!");

    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Reading info from CoppeliaSim...");
    const int iter1 = 2;
    for (int i=0; i<iter1;i++)
    {
        try
        {
            auto jointnames = impl_->robot_cs_->get_jointnames();
            jointnames.pop_back();

            //q = impl_->robot_cs_->get_configuration();
            q = DQ_robotics_extensions::Numpy::vstack(impl_->robot_client_->get_b1_pose().vec8(),
                                                      impl_->robot_client_->get_arm_joint_states());
            DQ xbase = DQ(q.head(8));
            VectorXd qarm = q.tail(6);
            offset = xbase.conj()*impl_->cs_->get_object_pose(jointnames.at(0));


            RCLCPP_INFO_STREAM(node_->get_logger(), "::Reading info from CoppeliaSim: "+std::to_string(i)+"/"+std::to_string(iter1));
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), std::string("::Exception::") + e.what());
            std::cerr << std::string("::Exception::") << e.what();
        }
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Data from CoppeliaSim OK!");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (_should_shutdown())
            break;
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Updating kinematic model...");
        /*********************************************
         * ROBOT KINEMATIC MODEL
         * *******************************************/
        auto arm = std::make_shared<DQ_SerialManipulatorDH>(UnitreeZ1Robot::kinematics());
        impl_->kin_mobile_manipulator_ = std::make_shared<CFFSerialRobot>(arm);
        impl_->kin_mobile_manipulator_->set_offset(offset);
        impl_->robot_model_ = std::shared_ptr<DQ_Kinematics>(impl_->kin_mobile_manipulator_);


        auto [Aeq_wm, beq_wm] = impl_->kin_mobile_manipulator_->get_six_dof_constraints(CFFSerialRobot::SIX_DOF_CONSTRAINT_MODE::PLANAR_JOINT);
        impl_->Aeq_wm_ = Aeq_wm;
        impl_->beq_wm_ = beq_wm;



        DQ x = impl_->robot_model_->fkm(q);
        impl_->cs_->set_object_pose(configuration_.cs_desired_frame, x);

        impl_->dim_control_inputs_ = 12;



        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Model updated!");
    }
}

ControlExample::~ControlExample()
{

}

/**
 * @brief ControlExample::ControlExample ctor of the class
 * @param node ROS2 node
 * @param configuration The configuration
 * @param break_loops The atomic boolean to break any loops when the user triggers the interruption signal
 */
ControlExample::ControlExample(std::shared_ptr<Node> &node,
                               const ControlExampleConfiguration &configuration,
                               std::atomic_bool *break_loops)
:
configuration_{configuration},
st_break_loops_{break_loops},
node_{node},
robot_reached_region_{false},
show_controller_idle_status_{true},
show_controller_on_status_{true}
{
    impl_ = std::make_unique<ControlExample::Impl>(node_,
                                                   configuration_.thread_sampling_time_sec,
                                                   configuration_.B1_topic_prefix);
    impl_->cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    impl_->robot_client_ = std::make_shared<UnitreeB1Z1RobotClient>(node_,
                                                                    configuration_.B1_topic_prefix,
                                                                    configuration_.Z1_topic_prefix,
                                                                    UnitreeB1Z1RobotClient::MODE::CONTROL);
}

/**
 * @brief ControlExample::control_loop
 */
void ControlExample::control_loop()
{
    impl_->clock_.init();
    rclcpp::spin_some(node_);


    /// If you want to set the desired pose using a topic, you need to wait for that topic before
    /// start the control loop. I commented the following lines, since for this example I am not waiting for a desired pose.
    /// In the following lines, I wait for the topic configuration_.B1_topic_prefix + "/get/coppeliasim_frame_xd"
    /*
    while (!impl_->new_coppeliasim_xd_data_available() && !_should_shutdown())
    {
        rclcpp::spin_some(node_);
        impl_->clock_.update_and_sleep();
        rclcpp::spin_some(node_);
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for desired pose from CoppeliaSim ");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    */

    // Wait for the robot drivers
    while (!impl_->robot_client_->is_enabled() && !_should_shutdown())
    {
        impl_->spin_and_update(std::string("Waiting for robot data"));
    }
    impl_->spin_and_update(std::string("Robot is ready!"));

    // Since both the drivers and the CoppeliaSim node is running, we can connect to CoppeliaSim
    // to update the kinematic model offsets.
    impl_->connect(configuration_.cs_host,
                   configuration_.cs_port,
                   configuration_.cs_TIMEOUT_IN_MILISECONDS,
                   configuration_.cs_B1_robotname,
                   configuration_.cs_Z1_robotname);
    _update_kinematic_model();

    //###########################################################################################################//
    //####################### Main Kinematic control loop running here ##########################################//

    for (int i=0;i<2000;i++) //---------??????????????????????????????
    {
        rclcpp::spin_some(node_);
        impl_->clock_.update_and_sleep();
        rclcpp::spin_some(node_);
        //RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Setting Forced stand mode ");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //impl_->robot_client_->set_stand_mode();
    }




    VectorXd qi_arm = impl_->robot_client_->get_arm_joint_states();
    [[maybe_unused]] unsigned int i = 0;
    const double& T = configuration_.thread_sampling_time_sec;

    //VectorXd q = impl_->robot_cs_->get_configuration();
    VectorXd q = DQ_robotics_extensions::Numpy::vstack(impl_->robot_client_->get_b1_pose().vec8(),
                                                       impl_->robot_client_->get_arm_joint_states());

    //---------------------------------
    /// controller settings
    const double& damping = configuration_.controller_damping;
    const double& gain = configuration_.controller_proportional_gain;
    auto solver_ = std::make_shared<DQ_QPOASESSolver>();
    MatrixXd A;
    VectorXd b;
    MatrixXd Aeq = impl_->Aeq_wm_;
    VectorXd beq = impl_->beq_wm_;

    //---------------------Robot constraint Manager


   auto [q_dot_min, q_dot_max] = configuration_.configuration_velocity_limits;

    VectorXd b_sat = VectorXd(24);
    b_sat << q_dot_max, -q_dot_min;

    MatrixXd A_sat = MatrixXd(24, 12);
    A_sat << MatrixXd::Identity(12,12), -MatrixXd::Identity(12,12);

    auto [q_min, q_max] = configuration_.configuration_limits;

    VectorXd qarm_min = q_min.tail(6);
    VectorXd qarm_max = q_max.tail(6);


    MatrixXd Aarm_config_min = MatrixXd(6,12);
    Aarm_config_min << MatrixXd::Zero(6,6), -MatrixXd::Identity(6,6);

    MatrixXd Aarm_config_max = MatrixXd(6,12);
    Aarm_config_max << MatrixXd::Zero(6,6), MatrixXd::Identity(6,6);
    double narm = 5.0;

    auto vfi_config_yaml = std::make_shared<DQ_robotics_extensions::VFIConfigurationFileYaml>();


    auto rcm = std::make_shared<DQ_robotics_extensions::RobotConstraintManager>(impl_->cs_,
                                                                                impl_->robot_cs_,
                                                                                impl_->robot_model_,
                                                                                vfi_config_yaml,
                                                                                configuration_.vfi_file,
                                                                                true);





    //rcm->set_configuration_limits({q_min, q_max});
    //rcm->set_configuration_velocity_limits({q_dot_min, q_dot_max});


    /*
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Waiting for a connection with sas_datalogger...");
    while(!impl_->datalogger_client_.is_enabled() && !_should_shutdown())
    {

    }
    RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Connected to sas_datalogger!");
*/

    VectorXd u;


    while(!_should_shutdown())
    {
        rclcpp::spin_some(node_);
        impl_->clock_.update_and_sleep();
        rclcpp::spin_some(node_);
        RCLCPP_INFO_ONCE(node_->get_logger(), "Control loop running!");
        //VectorXd q = DQ_robotics_extensions::Numpy::vstack(DQ_robotics_extensions::get_planar_joint_configuration_from_pose(impl_->robot_client_->get_b1_pose()),
        //                           qi_arm);

        VectorXd q = DQ_robotics_extensions::Numpy::vstack(impl_->robot_client_->get_b1_pose().vec8(), qi_arm);
        DQ x = impl_->robot_model_->fkm(q);
        DQ xd = x;
        impl_->publish_coppeliasim_frame_x(x);
        if (impl_->new_coppeliasim_xd_data_available())
        {
            // There is new data, then we update the desired pose;
            xd = impl_->get_desired_pose();
        }else{
            // There is no new data about xd, then the desired pose is the same
            // as the current pose. Consequently, the error is zero and robot stops.
            xd = x;
        }
        impl_->publish_coppeliasim_frame_x(x);


        VectorXd error_1 =  vec8( x.conj()*xd - 1 );
        VectorXd error_2 =  vec8( x.conj()*xd + 1 );
        double norm_1 = error_1.norm();
        double norm_2 = error_2.norm();

        if (norm_1 > norm_2){
            xd = -xd;
        }
        DQ error = x-xd;

        //------------------------
        MatrixXd J = impl_->robot_model_->pose_jacobian(q);
        MatrixXd H_aux = J.transpose()*J;
        MatrixXd H = H_aux + MatrixXd::Identity(H_aux.cols(), H_aux.cols())*damping*damping;
        VectorXd f = gain*2*J.transpose()*vec8(error);
        //------------------------------------------

        ///------------------
        double alpha = 0.99;
        VectorXd et = vec4(x.translation() - xd.translation());


        VectorXd er = _get_rotation_error(x, xd);

        const MatrixXd& Jx = J;
        DQ rd = xd.rotation();
        MatrixXd Jr = DQ_Kinematics::rotation_jacobian(Jx);
        MatrixXd Nr = haminus4(rd)*C4()*Jr;
        MatrixXd Jt = DQ_Kinematics::translation_jacobian(Jx, x);



        MatrixXd Ht = Jt.transpose()*Jt;
        VectorXd ft = gain*Jt.transpose()*et;

        MatrixXd Hr = Nr.transpose()*Nr;
        VectorXd fr = gain*Nr.transpose()*er;

        MatrixXd Hd = MatrixXd::Identity(Ht.cols(), Ht.cols())*damping*damping;

        MatrixXd H2 = alpha*Ht + (1.0 - alpha)*Hr + Hd;
        VectorXd f2 = alpha*ft + (1.0 - alpha)*fr;

        try {
            ///-------------------------------------------------------------------

            rcm->add_inequality_constraint(Aarm_config_min,  narm*(qi_arm-qarm_min)); //arm configuration
            rcm->add_inequality_constraint(Aarm_config_max, -narm*(qi_arm-qarm_max)); //arm configuration
            rcm->add_inequality_constraint(A_sat,  b_sat); //arm configuration

            auto constraints = rcm->get_inequality_constraints(q,false, false);
            A = std::get<0>(constraints);
            b = std::get<1>(constraints);

            u = solver_->solve_quadratic_program(H,f,A,b,Aeq,beq);
            // std::cout<<"u: "<<u.transpose()<<std::endl;
            ///-------------------------------------------------------------------
        } catch (const std::exception& e) {
            RCLCPP_INFO_STREAM(node_->get_logger(), "::QP not solved!");
            RCLCPP_INFO_STREAM(node_->get_logger(), e.what());
            u = VectorXd::Zero(impl_->dim_control_inputs_);
        }

        //Compute the distance between the end-effector and desired points
        double distance = (x.translation()-xd.translation()).vec3().norm();




        if (distance <= configuration_.controller_target_region_size)
        {
            robot_reached_region_ = true;
        }
        if (distance > configuration_.controller_target_exit_size)
        {
            robot_reached_region_ = false;
        }

        if (robot_reached_region_)
        {
            u = VectorXd::Zero(impl_->dim_control_inputs_);
            if (show_controller_idle_status_)
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "::Reached target zone. No risk of collision. ZERO mode enabled!");
                show_controller_idle_status_ = false;
                show_controller_on_status_ = true;
            }
        }else
        {
            if (show_controller_on_status_)
            {
                if (!robot_reached_region_)
                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Robot left the target zone. Motion mode enabled!");
                show_controller_on_status_ = false;
                show_controller_idle_status_ = true;
            }
        }




        DQ twist_u = DQ(u.head(6));
        VectorXd uarm = u.tail(6);

        //Numerical integration
        qi_arm = qi_arm + T*uarm;




    //    impl_->robot_client_->set_forced_stand_commands(rpy.x(),rpy.y(),rpy.z(), 0.0);
        impl_->robot_client_->set_arm_joint_positions(DQ_robotics_extensions::Numpy::vstack(qi_arm, DQ_robotics_extensions::CVectorXd({0.0})));
        //impl_->robot_client_->set_target_b1_twist(twist_u);
        VectorXd twist_u_vec = twist_u.vec6();
        VectorXd planar_vel = (VectorXd(3) << twist_u_vec(3), twist_u_vec(4), twist_u_vec(2)).finished();
        impl_->robot_client_->set_target_b1_planar_joint_velocities(planar_vel);

        i++;

    }


}





}
