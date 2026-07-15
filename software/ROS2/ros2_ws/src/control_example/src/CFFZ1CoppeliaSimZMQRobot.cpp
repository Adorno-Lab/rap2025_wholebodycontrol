#include "CFFZ1CoppeliaSimZMQRobot.h"

/**
* @brief _vstack stacks matrices in sequence vertically
* @param A
* @param B
* @return The matrix [A;
*                     B]
*/
MatrixXd _vstack(const MatrixXd &A, const MatrixXd &B);


MatrixXd _vstack(const MatrixXd &A, const MatrixXd &B)
{
    int m_A = A.rows();
    int m_B = B.rows();
    int n_A = A.cols();
    int n_B = B.cols();

    if (n_A != n_B)
        throw std::runtime_error(std::string("Wrong call of _vstack(A, B). ")
                                 + std::string("Incompatible sizes. The cols of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(A.rows())+ std::string("x")+ std::to_string(n_A)
                                 + std::string(" and B is ")+ std::to_string(B.rows()) + std::string("x")+ std::to_string(n_B));

    MatrixXd C = MatrixXd::Zero(m_A + m_B, n_A);
    C.block(0,0, m_A, n_A) = A;
    C.block(m_A, 0, m_B, n_B) = B;
    return C;
}



CFFZ1CoppeliaSimZMQRobot::
    CFFZ1CoppeliaSimZMQRobot(const std::string &robot_name,
                             const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &coppeliasim_interface_sptr)
:DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface_sptr)
{
    _initialize_robot_objectnames_from_coppeliasim();
}

std::vector<std::string> CFFZ1CoppeliaSimZMQRobot::get_jointnames() const
{
    return jointnames_;
}

/**
 * @brief CFFZ1CoppeliaSimZMQRobot::set_configuration
 * @param q The desired configuration q = [vec8(xbase); qarm]
 */
void CFFZ1CoppeliaSimZMQRobot::set_configuration(const VectorXd &q)
{
    const VectorXd qbase = q.head(8);
    const VectorXd qarm  = q.tail(6);

    const DQ xbase = DQ(qbase);
    _get_interface_sptr()->set_object_pose(holonomic_base_name_, xbase);
    _get_interface_sptr()->set_joint_positions(jointnames_, qarm);
}

/**
 * @brief CFFZ1CoppeliaSimZMQRobot::get_configuration
 * @return The robot configuration q = [vec8(xbase); qarm]
 */
VectorXd CFFZ1CoppeliaSimZMQRobot::get_configuration()
{
    VectorXd qbase = _get_base_pose().vec8();
    VectorXd q = _vstack(qbase, _get_joint_arm_positions());
    return q;
}

void CFFZ1CoppeliaSimZMQRobot::set_target_configuration([[maybe_unused]] const VectorXd &q_target)
{
    throw std::runtime_error("Unsupported");
}

VectorXd CFFZ1CoppeliaSimZMQRobot::get_configuration_velocities()
{
    throw std::runtime_error("Unsupported");
}

void CFFZ1CoppeliaSimZMQRobot::set_target_configuration_velocities([[maybe_unused]] const VectorXd &v_target)
{
    throw std::runtime_error("Unsupported");
}

void CFFZ1CoppeliaSimZMQRobot::set_target_configuration_forces([[maybe_unused]] const VectorXd &t)
{
    throw std::runtime_error("Unsupported");
}

VectorXd CFFZ1CoppeliaSimZMQRobot::get_configuration_forces()
{
    throw std::runtime_error("Unsupported");
}


void CFFZ1CoppeliaSimZMQRobot::_initialize_robot_objectnames_from_coppeliasim()
{
    alljointnames_ = _get_interface_sptr()->get_jointnames_from_object(robot_name_+"/UnitreeZ1");
    jointnames_ = alljointnames_;
    jointnames_.pop_back();
    gripper_jointname_ = alljointnames_.back();
    holonomic_base_name_ = robot_name_; //+"/trunk_respondable";
}

VectorXd CFFZ1CoppeliaSimZMQRobot::_get_joint_arm_positions()
{
    return _get_interface_sptr()->get_joint_positions(jointnames_);
}

DQ CFFZ1CoppeliaSimZMQRobot::_get_base_pose()
{
   return _get_interface_sptr()->get_object_pose(holonomic_base_name_);
}

