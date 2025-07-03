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

#pragma once
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <memory>

namespace DQ_robotics
{

class UnitreeB1Z1MobileRobot: public DQ_Kinematics
{
protected:
    std::shared_ptr<DQ_SerialManipulatorDH> kin_arm_;
    std::shared_ptr<DQ_HolonomicBase> kin_holonomic_base_;
    const int dim_configuration_space_ = 9;
    DQ X_J1_OFFSET_;
    DQ X_HEIGHT_OFFSET_;

public:
    UnitreeB1Z1MobileRobot();


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

    void update_base_offset(const DQ& X_J1_OFFSET);
    void update_base_height_from_IMU(const DQ& X_IMU);

};

}


