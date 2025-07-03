#include "UnitreeB1Z1CoppeliaSimZMQRobot.hpp"
#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>

UnitreeB1Z1CoppeliaSimZMQRobot::UnitreeB1Z1CoppeliaSimZMQRobot(const std::string &robot_name,
                                                               const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &coppeliasim_interface_sptr)
    :DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface_sptr)
{
    _initialize_robot_objectnames_from_coppeliasim();
}


/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::_initialize_robot_objectnames_from_coppeliasim
 */
void UnitreeB1Z1CoppeliaSimZMQRobot::_initialize_robot_objectnames_from_coppeliasim()
{
    alljointnames_ = _get_interface_sptr()->get_jointnames_from_object(robot_name_+"/UnitreeZ1");
    jointnames_ = alljointnames_;
    jointnames_.pop_back();
    gripper_jointname_ = alljointnames_.back();
    holonomic_base_name_ = robot_name_; //+"/trunk_respondable";
    //height_joint_ = robot_name_+"/height_joint";
    //base_offset_ = _get_interface_sptr()->get_object_pose(jointnames_.at(0));

}

VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::_get_mobile_robot_configuration_from_pose(const DQ &base)
{
    auto x = base;
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}

VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::_get_joint_arm_positions()
{
    return _get_interface_sptr()->get_joint_positions(jointnames_);
}


DQ UnitreeB1Z1CoppeliaSimZMQRobot::_get_base_pose()
{
 return _get_interface_sptr()->get_object_pose(holonomic_base_name_);
}

void UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration([[maybe_unused]] const VectorXd &q)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration: Unsupported");
}

VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration()
{
    VectorXd qbase = _get_mobile_robot_configuration_from_pose(_get_base_pose());
    VectorXd q = DQ_robotics_extensions::Numpy::vstack(qbase, _get_joint_arm_positions());
    return q;
}

void UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration([[maybe_unused]] const VectorXd &q_target)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration: Unsupported");
}

VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_velocities()
{
    throw std::runtime_error("UnitreeeB1Z1CoppeliaSimZMQRobot::get_configuration_velocities: Unsupported");
    return VectorXd::Zero(0);
}

void UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_velocities([[maybe_unused]] const VectorXd &v_target)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_space_velocities: Unsupported");
}

void UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_forces([[maybe_unused]] const VectorXd &t)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration_space_torques: Unsupported");
}

VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_forces()
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_space_torques: Unsupported");
    return VectorXd::Zero(0);
}




