#pragma once
#include <memory>
#include <thread>
#include <atomic>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>
#include <iostream>

class RobotDriverCoppeliaSimUnitreeB1Z1
{
private:
    enum STATUS{
        IDLE,
        CONNECTED,
        INITIALIZED,
        DEINITIALIZED,
        DISCONNECTED,
    };
    STATUS current_status_{IDLE};



    void _update_coppeliasim();

    void _echo_robot_state_mode();
    std::thread echo_robot_state_mode_thread_;
    void _start_echo_robot_state_mode_thread();
    std::atomic<bool> finish_echo_robot_state_;


protected:
    std::atomic_bool* break_loops_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> cs_;
    std::string ip_;
    std::vector<std::string> jointnames_;
    double simulation_time_{0};
    int port_;
    std::string status_msg_;
    int timeout_;

protected:
    std::string b1_name_;
    std::string z1_name_;
    std::string gripper_joint_name_;

    VectorXd q_ = VectorXd::Zero(9);
    DQ robot_b1_pose_{1};
    DQ b1z1_frame_1{1};
    DQ b1z1_frame_0{1};
    DQ b1z1_frame_2{1};
    DQ b1z1_frame_3{1};
    DQ b1z1_frame_4{1};

    std::string FR_hip_rotor_{"FR_hip_rotor_joint"};
    std::string FL_hip_rotor_{"FL_hip_rotor_joint"};
    std::string RR_hip_rotor_{"RR_hip_rotor_joint"};
    std::string RL_hip_rotor_{"RL_hip_rotor_joint"};
    std::vector<std::string> FR_jointnames_;
    std::vector<std::string> FL_jointnames_;
    std::vector<std::string> RR_jointnames_;
    std::vector<std::string> RL_jointnames_;
    //-------------------------------------------------------------
    //---------------Robot commands------------------------
    //----joint positions--(unit: radian)
    VectorXd qFR_cmd_  = VectorXd::Zero(3);
    VectorXd qFL_cmd_  = VectorXd::Zero(3);
    VectorXd qRL_cmd_  = VectorXd::Zero(3);
    VectorXd qRR_cmd_  = VectorXd::Zero(3);
    VectorXd qarm_cmd_ = VectorXd::Zero(6);
    double q_gripper_{0};


    DQ x_{1};
    DQ xd_{1};
    DQ pose_of_the_first_arm_joint_{1};
    DQ x_marker_1_{1};

    bool update_desired_pose_{false};
    bool update_effector_frame_ = {false};
    bool update_gripper_position_{false};
    bool update_b1_pose_{false};
    bool update_leg_positions_{false};
    bool update_arm_positions_{false};
    bool update_b1z1_frame_1_pose_{false};
    bool update_b1z1_frame_0_pose_{false};
    bool update_b1z1_frame_2_pose_{false};
    bool update_b1z1_frame_3_pose_{false};
    bool update_b1z1_frame_4_pose_{false};

    bool update_b1z1_frame_0_plane_{false};
    bool update_b1z1_frame_2_plane_{false};
    bool update_b1z1_frame_3_plane_{false};
    bool update_b1z1_frame_4_plane_{false};

public:
    RobotDriverCoppeliaSimUnitreeB1Z1( std::atomic_bool* break_loops,
                                      const std::string& B1_robotname,
                                      const std::string& Z1_robotname,
                                      const std::string& host = "localhost",
                                      const int& port = 23000,
                                      const int&TIMEOUT_IN_MILISECONDS = 1000);

    void set_leg_positions(const std::tuple<VectorXd, VectorXd, VectorXd, VectorXd>& u_commands);
    void set_arm_positions(const VectorXd& qarm);
    VectorXd get_arm_positions() const;

    double get_gripper_position() const;
    void set_gripper_position(const double& gripper_position);

    void set_effector_frame_pose(const DQ& x);

    DQ get_desired_frame_pose() const;
    void update_desired_pose(const DQ& xd);

    void set_IMU_pose(const DQ& x);
    DQ get_IMU_pose() const;

    DQ get_pose_of_the_first_arm_joint() const;

    void set_b1z1_frame_0(const DQ& pose);
    void set_b1z1_frame_1(const DQ& pose);
    void set_b1z1_frame_2(const DQ& pose);
    void set_b1z1_frame_3(const DQ& pose);
    void set_b1z1_frame_4(const DQ& pose);

    DQ get_pose_of_marker_1() const;




    VectorXd get_mobile_platform_configuration_from_IMU_pose() const;

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();
};


