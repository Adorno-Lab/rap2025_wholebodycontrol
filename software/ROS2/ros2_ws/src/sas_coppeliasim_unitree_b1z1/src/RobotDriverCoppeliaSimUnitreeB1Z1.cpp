#include "RobotDriverCoppeliaSimUnitreeB1Z1.hpp"


/**
 * @brief RobotDriverCoppeliaSimUnitreeB1Z1::RobotDriverCoppeliaSimUnitreeB1Z1
 * @param B1_robotname
 * @param Z1_robotname
 * @param host
 * @param port
 * @param TIMEOUT_IN_MILISECONDS
 */
RobotDriverCoppeliaSimUnitreeB1Z1::RobotDriverCoppeliaSimUnitreeB1Z1(std::atomic_bool *break_loops,
                                                                     const std::string &B1_robotname,
                                                                     const std::string &Z1_robotname,
                                                                     const std::string &host,
                                                                     const int &port,
                                                                     const int &TIMEOUT_IN_MILISECONDS)
:break_loops_{break_loops}, ip_{host}, port_{port}, timeout_{TIMEOUT_IN_MILISECONDS}, b1_name_{B1_robotname}, z1_name_{Z1_robotname}
{
    cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
}

/**
 * @brief RobotDriverCoppeliaSimUnitreeB1Z1::connect
 */
void RobotDriverCoppeliaSimUnitreeB1Z1::connect()
{
    if (cs_)
    {
        try
        {
            if (!cs_->connect(ip_, port_, timeout_))
                throw std::runtime_error("Unable to connect to CoppeliaSim.");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::cerr<<"Connected to CoppeliaSim! "<<std::endl;
            status_msg_ = "connected!";
            current_status_ = STATUS::CONNECTED;
            // Get the joint names from CoppeliaSim
            auto jointnames_aux = cs_->get_jointnames_from_object(z1_name_);
            jointnames_ = jointnames_aux;
            jointnames_.pop_back();
            gripper_joint_name_ = jointnames_aux.back();

            FR_jointnames_ = cs_->get_jointnames_from_object(FR_hip_rotor_);
            FL_jointnames_ = cs_->get_jointnames_from_object(FL_hip_rotor_);
            RR_jointnames_ = cs_->get_jointnames_from_object(RR_hip_rotor_);
            RL_jointnames_ = cs_->get_jointnames_from_object(RL_hip_rotor_);
            robot_b1_pose_ = cs_->get_object_pose(b1_name_);


        }
        catch (std::exception& e)
        {
            std::cout<<e.what()<<std::endl;
            current_status_ = STATUS::IDLE;
            status_msg_ = "Unable to connect to CoppeliaSim.";
        }
    }
}

/**
 * @brief RobotDriverCoppeliaSimUnitreeB1Z1::initialize
 */
void RobotDriverCoppeliaSimUnitreeB1Z1::initialize()
{
    if (current_status_ == STATUS::CONNECTED)
    {
        status_msg_ = "Initialized!";
        current_status_ = STATUS::INITIALIZED;
        _start_echo_robot_state_mode_thread();
    }else{
        //std::cerr<<"The driver must be connected before to be initialized. "<<std::endl;
        status_msg_ = "Unable to initialize.";
    }
}

void RobotDriverCoppeliaSimUnitreeB1Z1::deinitialize()
{
    if (current_status_ == STATUS::INITIALIZED)
    {

        status_msg_ = "Finishing echo robot state.";
        finish_echo_robot_state_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (echo_robot_state_mode_thread_.joinable())
            echo_robot_state_mode_thread_.join();
        current_status_ = STATUS::DEINITIALIZED;
        status_msg_ = "Deinitialized!";
    }
}

void RobotDriverCoppeliaSimUnitreeB1Z1::disconnect()
{
    current_status_ = STATUS::DISCONNECTED;
    status_msg_ = "Deinitialized!";
}

std::string RobotDriverCoppeliaSimUnitreeB1Z1::get_status_message() const
{
    return status_msg_;
}

std::string RobotDriverCoppeliaSimUnitreeB1Z1::get_ip() const
{
    return ip_;
}

int RobotDriverCoppeliaSimUnitreeB1Z1::get_port() const
{
    return port_;
}


/**
 * @brief RobotDriverCoppeliaSimUnitreeB1Z1::_update_coppeliasim
 */
void RobotDriverCoppeliaSimUnitreeB1Z1::_update_coppeliasim()
{
    q_ = cs_->get_joint_positions(jointnames_);

    if (is_unit(x_) and update_effector_frame_)
    {
        cs_->set_object_pose("x", x_);
        update_effector_frame_ = false;
    }
    xd_ = cs_->get_object_pose("xd");
    pose_of_the_first_arm_joint_ = cs_->get_object_pose(jointnames_.at(0));

    if (update_b1_pose_ and is_unit(robot_b1_pose_))
    {
        cs_->set_object_pose(b1_name_, robot_b1_pose_);
        update_b1_pose_ = false;
    }
    if (update_desired_pose_ and is_unit(xd_))
    {
        cs_->set_object_pose("xd", xd_);
        update_desired_pose_ = false;
    }
    if (update_gripper_position_)
    {
        cs_->set_joint_positions({gripper_joint_name_}, (VectorXd(1)<<q_gripper_).finished());
        update_gripper_position_ = false;
    }
    if (update_leg_positions_)
    {
        cs_->set_joint_positions(FR_jointnames_, qFR_cmd_);
        cs_->set_joint_positions(FL_jointnames_, qFL_cmd_);
        cs_->set_joint_positions(RR_jointnames_, qRR_cmd_);
        cs_->set_joint_positions(RL_jointnames_, qRL_cmd_);
        update_leg_positions_ = false;
    }
    if (update_arm_positions_)
    {
        cs_->set_joint_positions(jointnames_, qarm_cmd_);
        update_arm_positions_ = false;
    }
    if (update_b1z1_frame_1_pose_ and is_unit(b1z1_frame_1))
    {
        cs_->set_object_pose("B1Z1_Frame_1", b1z1_frame_1);
        update_b1z1_frame_1_pose_ = false;
    }

    if (update_b1z1_frame_0_pose_ and is_unit(b1z1_frame_0))
    {
        cs_->set_object_pose("B1Z1_Frame_0", b1z1_frame_0);
        if (update_b1z1_frame_0_plane_)
        {
            VectorXd p = b1z1_frame_0.translation().vec3();
            p(1) = 0;

            cs_->set_object_translation("/Plane_frame_0", b1z1_frame_0.translation());
            update_b1z1_frame_0_plane_ = false;
        }
        update_b1z1_frame_0_pose_ = false;
    }

    if (update_b1z1_frame_2_pose_ and is_unit(b1z1_frame_2))
    {
        cs_->set_object_pose("B1Z1_Frame_2", b1z1_frame_2);
        cs_->set_object_translation("/foam", b1z1_frame_2.translation());
        if (update_b1z1_frame_2_plane_)
        {
            cs_->set_object_translation("/Plane_frame_2", b1z1_frame_2.translation());
            update_b1z1_frame_2_plane_ = false;
        }
        update_b1z1_frame_2_pose_ = false;
    }

    if (update_b1z1_frame_3_pose_ and is_unit(b1z1_frame_3))
    {
        cs_->set_object_pose("B1Z1_Frame_3", b1z1_frame_3);
        if (update_b1z1_frame_3_plane_)
        {
            cs_->set_object_translation("/Plane_frame_3", b1z1_frame_3.translation());
            update_b1z1_frame_3_plane_ = false;
        }
        update_b1z1_frame_3_pose_ = false;
    }

    if (update_b1z1_frame_4_pose_ and is_unit(b1z1_frame_4))
    {
        cs_->set_object_pose("B1Z1_Frame_4", b1z1_frame_4);
        if (update_b1z1_frame_4_plane_)
        {
            cs_->set_object_translation("/Plane_frame_4", b1z1_frame_4.translation());
            update_b1z1_frame_4_plane_ = false;
        }
        update_b1z1_frame_4_pose_ = false;
    }

    x_marker_1_ = cs_->get_object_pose("/UnitreeB1/B1Z1_vicon_marker_1");
    VectorXd q_gripper = cs_->get_joint_positions({gripper_joint_name_});
    q_gripper_ = q_gripper(0);
    //std::this_thread::sleep_for(std::chrono::milliseconds(4));
}


/**
 * @brief RobotDriverCoppeliaSimUnitreeB1Z1::_echo_robot_state_mode
 */
void RobotDriverCoppeliaSimUnitreeB1Z1::_echo_robot_state_mode()
{
    while(!finish_echo_robot_state_ or !break_loops_)
    {
        try
        {
            _update_coppeliasim();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        catch (std::exception& e)
        {
            std::cout<<e.what()<<std::endl;
        }
    }
}

void RobotDriverCoppeliaSimUnitreeB1Z1::_start_echo_robot_state_mode_thread()
{
    finish_echo_robot_state_ = false;
    status_msg_ = "Checking echo robot state thread";
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    status_msg_ ="Starting echo robot state thread";
    echo_robot_state_mode_thread_ = std::thread(&RobotDriverCoppeliaSimUnitreeB1Z1::_echo_robot_state_mode, this);
}



void RobotDriverCoppeliaSimUnitreeB1Z1::set_leg_positions(const std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> &u_commands)
{
    auto [uFR, uFL, uRR, uRL] = u_commands;
    qFR_cmd_ = uFR;
    qFL_cmd_ = uFL;
    qRR_cmd_ = uRR;
    qRL_cmd_ = uRL;
    update_leg_positions_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_arm_positions(const VectorXd &qarm)
{
    qarm_cmd_ = qarm;
    update_arm_positions_ = true;
}

VectorXd RobotDriverCoppeliaSimUnitreeB1Z1::get_arm_positions() const
{
    return q_;
}

double RobotDriverCoppeliaSimUnitreeB1Z1::get_gripper_position() const
{
    return q_gripper_;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_gripper_position(const double &gripper_position)
{
    q_gripper_ = gripper_position;
    update_gripper_position_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_effector_frame_pose(const DQ &x)
{
    x_ = x.normalize();
    update_effector_frame_ = true;
}

DQ RobotDriverCoppeliaSimUnitreeB1Z1::get_desired_frame_pose() const
{
    return xd_.normalize();
}

void RobotDriverCoppeliaSimUnitreeB1Z1::update_desired_pose(const DQ &xd)
{
    xd_ = xd.normalize();
    update_desired_pose_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_IMU_pose(const DQ &x)
{
    robot_b1_pose_ = x.normalize();
    update_b1_pose_ = true;
}

DQ RobotDriverCoppeliaSimUnitreeB1Z1::get_IMU_pose() const
{
    return robot_b1_pose_;
}


DQ RobotDriverCoppeliaSimUnitreeB1Z1::get_pose_of_the_first_arm_joint() const
{
    return pose_of_the_first_arm_joint_;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_b1z1_frame_0(const DQ &pose)
{
    b1z1_frame_0 = pose;
    update_b1z1_frame_0_pose_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_b1z1_frame_1(const DQ &pose)
{
    b1z1_frame_1 = pose;
    update_b1z1_frame_1_pose_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_b1z1_frame_2(const DQ &pose)
{
    b1z1_frame_2 = pose;
    update_b1z1_frame_2_pose_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_b1z1_frame_3(const DQ &pose)
{
    b1z1_frame_3 = pose;
    update_b1z1_frame_3_pose_ = true;
}

void RobotDriverCoppeliaSimUnitreeB1Z1::set_b1z1_frame_4(const DQ &pose)
{
    b1z1_frame_4 = pose;
    update_b1z1_frame_4_pose_ = true;
}

DQ RobotDriverCoppeliaSimUnitreeB1Z1::get_pose_of_marker_1() const
{
    return x_marker_1_;
}



VectorXd RobotDriverCoppeliaSimUnitreeB1Z1::get_mobile_platform_configuration_from_IMU_pose() const
{
    auto x = get_IMU_pose();
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}

