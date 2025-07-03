#pragma once
#include <vector>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h>



class UnitreeB1Z1CoppeliaSimZMQRobot: public DQ_CoppeliaSimRobotZMQ
{

protected:
    std::vector<std::string> alljointnames_;
    std::vector<std::string> jointnames_;
    std::string gripper_jointname_;
    std::string holonomic_base_name_;
    //std::string height_joint_;
    //DQ base_offset_{1};
    void _initialize_robot_objectnames_from_coppeliasim();
    VectorXd _get_mobile_robot_configuration_from_pose(const DQ &base);
    VectorXd _get_joint_arm_positions();
    DQ _get_base_pose();
public:
    UnitreeB1Z1CoppeliaSimZMQRobot(const std::string& robot_name,
                                  const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& coppeliasim_interface_sptr);





    void set_configuration(const VectorXd& q) override;
    VectorXd get_configuration() override;

    void set_target_configuration(const VectorXd& q_target) override;

    VectorXd get_configuration_velocities() override;
    void set_target_configuration_velocities(const VectorXd& v_target) override;

    void set_target_configuration_forces(const VectorXd& t) override;
    VectorXd get_configuration_forces() override;


};
