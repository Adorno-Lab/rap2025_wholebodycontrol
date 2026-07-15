#include "CFFZ1Robot.h".h"
#include "dqrobotics/robots/UnitreeZ1Robot.h"

namespace DQ_robotics
{


/**
 * @brief _hstack stacks matrices in sequence horizontally
 * @param A
 * @param B
 * @return The matrix [A B]
 */
MatrixXd _hstack(const MatrixXd &A, const MatrixXd &B);

/**
 * @brief _resize resizes a matrix A to a larger matrix of size (rowsxcols) that containts
 *               the matrix A. The additional elements are zeros.
 * @param A
 * @param rows
 * @param cols
 * @return The matrix [A 0
 *                     0 0]
 */
MatrixXd _resize(const MatrixXd &A, const int &rows, const int &cols);



///////////////////////////////////////////////

MatrixXd _resize(const MatrixXd &A, const int &rows, const int &cols)
{
    MatrixXd aux = MatrixXd::Zero(rows, cols);
    int m = A.rows();
    int n = A.cols();

    if (m > rows)
    {
        throw std::runtime_error(std::string("The rows you used is smaller than the rows of the Matrix. ")
                                 +std::string("Incompatible rows for resize. Matrix A has ")
                                 +std::to_string(m)+ std::string(" rows. But you used ")
                                 +std::to_string(rows));
    }
    if (n > cols)
    {
        throw std::runtime_error(std::string("The cols you used is smaller than the cols of the Matrix. ")
                                 +std::string("Incompatible cols for resize. Matrix A has ")
                                 +std::to_string(n)+ std::string(" cols. But you used ")
                                 +std::to_string(cols));
    }

    aux.block(0,0, m, n) = A;
    return aux;
}

MatrixXd _hstack(const MatrixXd &A, const MatrixXd &B)
{

    int m_A = A.rows();
    int m_B = B.rows();

    if (m_A != m_B)
        throw std::runtime_error(std::string("Wrong usage of _hstack(A, B). ")
                                 + std::string("Incompatible sizes. The rows of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(m_A)+ std::string("x")+ std::to_string(A.cols())
                                 + std::string(" and B is ")+ std::to_string(m_B) + std::string("x")+ std::to_string(B.cols()));
    int n_A = A.cols();
    int n_B = B.cols();
    MatrixXd C = MatrixXd::Zero(m_A, n_A + n_B);
    C.block(0,0, m_A, n_A) = A;
    C.block(0, n_A, m_B, n_B) = B;
    return C;
}



CFFZ1Robot::CFFZ1Robot() {

    kin_arm_ = std::make_shared<DQ_SerialManipulatorDH>(UnitreeZ1Robot::kinematics());

    for (auto i=0;i<3;i++){
        I_(i+1,i) = 1.0;
        I_(i+5,i+3) = 1.0;
    }
}

void CFFZ1Robot::set_offset(const DQ &x_b_a)
{
    x_b_a_ = x_b_a;
}

DQ CFFZ1Robot::fkm(const VectorXd &q) const
{
    return fkm(q, get_dim_configuration_space()-1);
}

/**
 * @brief CFFZ1Robot::pose_jacobian
 * @param q The configuration q = [vec8(xbase); qarm]'
 * @param ith
 * @return
 */
MatrixXd CFFZ1Robot::pose_jacobian(const VectorXd &q, const int &ith) const
{
    VectorXd qbase = q.head(8);
    VectorXd qarm  = q.tail(6);

    MatrixXd J;

    const DQ x_0_b = DQ(qbase);
    DQ x_a_e = kin_arm_->fkm(qarm);
    DQ x_b_e = x_b_a_*x_a_e;
    DQ x_0_a = x_0_b*x_b_a_;
    MatrixXd Jtwist = 0.5*hamiplus8(x_0_b)*I_;
    //MatrixXd Jarm = kin_arm_->pose_jacobian(qarm);



    if (ith < min_index_ or ith > max_index_)
        throw std::runtime_error("CFFZ1Robot::pose_jacobian: The minimum index is "+std::to_string(min_index_)+ ". "
                                  "The maximum index is "+std::to_string(max_index_)+ ". "
                                  "However, you used "+ std::to_string(ith));
    else if (ith == min_index_)
    {
        J = _resize(Jtwist, 8, get_dim_configuration_space());
    }else
    {
        MatrixXd Jarm = kin_arm_->pose_jacobian(qarm,  ith-(min_index_+1));
        J = _hstack(haminus8(x_b_e)*Jtwist, hamiplus8(x_0_a)*Jarm);
        if (J.cols() != get_dim_configuration_space())
            J = _resize(J, 8, get_dim_configuration_space());
    }

    return J;
}

MatrixXd CFFZ1Robot::pose_jacobian(const VectorXd &q) const
{
   return pose_jacobian(q, get_dim_configuration_space()-1);
}

int CFFZ1Robot::get_dim_configuration_space() const
{
    return dim_configuration_space_;
}

MatrixXd CFFZ1Robot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    throw std::runtime_error("pose_jacobian_derivative is not implemented yet.");
}

MatrixXd CFFZ1Robot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}


/*
DQ CFFManipulator::fkm(const DQ &x_base, const VectorXd &qarm) const
{
    const DQ& x_0_b = x_base; //aliasing
    DQ x_a_e = kin_arm_->fkm(qarm);
    return x_0_b*x_b_a_*x_a_e;
}

MatrixXd CFFManipulator::pose_jacobian(const DQ &x_base, const VectorXd &qarm) const
{
    const DQ& x_0_b = x_base; //aliasing
    DQ x_a_e = kin_arm_->fkm(qarm);
    DQ x_b_e = x_b_a_*x_a_e;
    DQ x_0_a = x_0_b*x_b_a_;
    MatrixXd Jtwist = 0.5*hamiplus8(x_0_b)*I_;
    MatrixXd Jarm = kin_arm_->pose_jacobian(qarm);

    MatrixXd J = _hstack(haminus8(x_b_e)*Jtwist, hamiplus8(x_0_a)*Jarm);
    return J;
}
*/


std::tuple<MatrixXd, VectorXd> CFFZ1Robot::get_six_dof_constraints(const SIX_DOF_CONSTRAINT_MODE &mode)
{

    MatrixXd Aeq = MatrixXd::Zero(3,12);
    VectorXd beq = VectorXd::Zero(3);
    switch (mode) {
    case SIX_DOF_CONSTRAINT_MODE::PLANAR_JOINT:
    {
        // Walking Mode (planar joint)
        Aeq << 1,0,0,0,0,0,      MatrixXd::Zero(1,6),
               0,1,0,0,0,0,      MatrixXd::Zero(1,6),
               0,0,0,0,0,1,      MatrixXd::Zero(1,6);
        break;
    }
    case SIX_DOF_CONSTRAINT_MODE::FORCED_STAND:
    {
        //Forced Stand Mode
        Aeq << MatrixXd::Zero(3,3),      MatrixXd::Identity(3,3) , MatrixXd::Zero(3,6);
        break;
    }
    case SIX_DOF_CONSTRAINT_MODE::ZERO:
    {
        Aeq = MatrixXd::Zero(6,12);
        beq = VectorXd::Zero(6);
        Aeq << MatrixXd::Identity(3,3),  MatrixXd::Zero(3,3),      MatrixXd::Zero(3,6),
               MatrixXd::Zero(3,3),      MatrixXd::Identity(3,3) , MatrixXd::Zero(3,6);
    }
    default:
        throw std::runtime_error("Wrong constraint mode!");
    }



    return {Aeq,beq};
}

/**
 * @brief CFFZ1Robot::fkm
 * @param q The configuration q = [vec8(xbase); qarm]'
 * @param ith
 * @return
 */
DQ CFFZ1Robot::fkm(const VectorXd &q, const int &ith) const
{
    const VectorXd qbase = q.head(8);
    const VectorXd qarm  = q.tail(6);
    DQ x;


    const DQ x_0_b = DQ(qbase);
   // DQ x_a_e = kin_arm_->fkm(qarm);
   // x_0_b*x_b_a_*x_a_e;




    if (ith < min_index_ or ith > max_index_)
        throw std::runtime_error("CFFZ1Robot::fkm: The minimum index is "+std::to_string(min_index_)+ ". "
                                 "The maximum index is "+std::to_string(max_index_)+ ". "
                                 "However, you used "+ std::to_string(ith));
    else if (ith == min_index_)
    {
        x = x_0_b;
    }else
    {
        x = x_0_b*x_b_a_*kin_arm_->fkm(qarm, ith-(min_index_+1));
    }
    return x;
}


}
