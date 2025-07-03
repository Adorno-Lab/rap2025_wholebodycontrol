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

#include "UnitreeZ1Robot.hpp"


namespace DQ_robotics
{

/**
 * @brief _get_dh_matrix gets the DH matrix of the Unitree Z1 manipulator
 * @return the DH matrix (including a row describing the joint types).
 */
MatrixXd _get_dh_matrix()
{
    const double pi = M_PI;
    Matrix<double,5,6> raw_dh_matrix(5,6);
    DQ_JointType R = DQ_JointType::REVOLUTE;
    raw_dh_matrix <<    0,      0,      pi/2+1.3151,  0.2557, -pi/2,  -pi/2, // theta
                    0.045,      0,                0,       0,     0, 0.0492, // d
                        0,  -0.35,         -0.22533,   -0.07,     0,      0, // a
                    -pi/2,      0,                0,   -pi/2,  pi/2,      0, // alpha
                        R,      R,                R,       R,     R,      R; // joint type
    return raw_dh_matrix;
}

/**
 * @brief _get_effector gets the end-effector offset.
 * @return A unit dual quaternion that represents the end-effector offset.
 */
DQ _get_effector()
{
    DQ effector_ = 1 + 0.5*E_*0.2*k_;
    return effector_;
}

/**
 * @brief UnitreeZ1Robot::kinematics returns the kinematic model of the robot.
 * @return An object of the DQ_SerialManipulatorDH class.
 */
DQ_SerialManipulatorDH UnitreeZ1Robot::kinematics()
{
    DQ_SerialManipulatorDH robot(_get_dh_matrix());
    robot.set_base_frame(DQ(1));
    robot.set_reference_frame(DQ(1));
    robot.set_effector(_get_effector());
    return robot;
}

}
