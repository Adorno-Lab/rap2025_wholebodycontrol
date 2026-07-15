#pragma once
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <memory>

namespace DQ_robotics
{


class CFFZ1Robot: public DQ_Kinematics
{
public:
    enum class SIX_DOF_CONSTRAINT_MODE{ZERO, PLANAR_JOINT, FORCED_STAND};
private:
    DQ x_b_a_{1};
    std::shared_ptr<DQ_SerialManipulatorDH> kin_arm_;
    const int dim_configuration_space_ = 12;
    MatrixXd I_ = MatrixXd::Zero(8,6);


    const int min_index_ = 5;
    const int max_index_ = 11;
public:
    CFFZ1Robot();
    void set_offset(const DQ& x_b_a);
    std::tuple<MatrixXd, VectorXd> get_six_dof_constraints(const SIX_DOF_CONSTRAINT_MODE& mode);

    /*


    DQ fkm(const DQ& x_base, const VectorXd& qarm) const;
    MatrixXd pose_jacobian(const DQ &x_base, const VectorXd& qarm) const ;

    int get_dim_configuration_space() const;
    std::tuple<MatrixXd, VectorXd> get_six_dof_constraints(const SIX_DOF_CONSTRAINT_MODE& mode);
    */



    //Abstract methods' implementation
    DQ fkm(const VectorXd& q, const int& ith) const override;
    DQ fkm(const VectorXd& q) const override;
    MatrixXd pose_jacobian(const VectorXd& q, const int& ith) const override;
    MatrixXd pose_jacobian(const VectorXd& q) const override;

    int get_dim_configuration_space() const override;

    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_ith_link) const override; //To be implemented.
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                      const VectorXd& q_dot) const override; //To be implemented.


        //roll pitch yaw in stand mode, roll range:[-0.3, 0.3], pitch range:[-0.3, 0.3], yaw range:[-0.6, 0.6]

};


}
