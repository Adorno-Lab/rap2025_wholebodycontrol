#pragma once
#include <QMainWindow>
#include <qspinbox.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/utils.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include <sas_core/eigen3_std_conversions.hpp>
#include <format>

namespace sas
{
struct DataConfiguration
{
    std::string B1_1_topic_prefix;
    std::string Z1_1_topic_prefix;
    std::string B1_2_topic_prefix;
    std::string Z1_2_topic_prefix;
    std::string task_commander_topic_prefix;
};
}

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(std::shared_ptr<rclcpp::Node> &node, const  sas::DataConfiguration &configuration, QWidget *parent = nullptr);
    ~MainWindow();

private slots:


    void _emergency_stop_button_clicked();
    void _pause_button_clicked();

private:
    std::vector<QDoubleSpinBox*> qarm_white_spinBoxes_;
    std::vector<QDoubleSpinBox*> qarm_black_spinBoxes_;
    std::vector<QDoubleSpinBox*> b1_pose_white_spinBoxes_;
    std::vector<QDoubleSpinBox*> u_white_spinBoxes_;
    std::vector<QDoubleSpinBox*> u_black_spinBoxes_;
    std::vector<QDoubleSpinBox*> b1_pose_black_spinBoxes_;
    std::vector<QDoubleSpinBox*> agent_3_pos_spinBoxes_;
    std::vector<QDoubleSpinBox*> agent_3_rot_spinBoxes_;
    std::vector<QDoubleSpinBox*> hcylinder_spinBoxes_;
    void _config_spin_boxes_as_read_only(const std::vector<QDoubleSpinBox*>& spinboxes);


    QString pause_yellow_color_ = "background-color: #fcca03;";
    QString disabled_black_color_ = "background-color: #6b6b6b; color: black;";
    QString active_green_color_ = "background-color: #0ad125;";

private:
    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *event);
    int timerId_;
    int time_step_in_milliseconds_;
    double elapsed_time_;
    bool enable_watchdog_;
    bool watchdog_notification_updated_;
    double watchdog_period_ = 3.0;

    sas::DataConfiguration configuration_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<sas::UnitreeB1Z1RobotClient> robot_client_white; // for monitoring, and shutdown
    std::shared_ptr<sas::UnitreeB1Z1RobotClient> robot_client_black; // for monitoring, and shutdown
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> white_twist_state_time_point_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> black_twist_state_time_point_;
    double white_twist_state_time_{0};
    double black_twist_state_time_{0};
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> white_z1_state_time_point_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> black_z1_state_time_point_;
    double white_z1_state_time_{0};
    double black_z1_state_time_{0};


    Publisher<sas_msgs::msg::Bool>::SharedPtr publisher_emergency_stop_device_signal_;


    void _callback_shutdown_switch(const sas_msgs::msg::Bool &msg);
    Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_shutdown_switch_;




    VectorXd control_inputs_white_;
    void _callback_control_inputs_white(const std_msgs::msg::Float64MultiArray &msg);
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_control_inputs_white_;

    VectorXd control_inputs_black_;
    void _callback_control_inputs_black(const std_msgs::msg::Float64MultiArray &msg);
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_control_inputs_black_;

    Publisher<std_msgs::msg::String>::SharedPtr test_publisher_;
    TimerBase::SharedPtr test_timer_;



    bool task_status_agent_1_;
    bool controller_status_agent_1_;
    double error_agent_1_;
    std::string status_msg_agent_1_;
    DQ xce1_;
    DQ yce1_;
    bool new_B1Z1_agent_1_output_available_{false};
  //  Subscription<consensus_protocol_msgs::msg::FormationControlAgentData>::SharedPtr subscriber_external_B1Z1_agent_data_1;
  //  void _callback_external_B1Z1_agent_1_output(const consensus_protocol_msgs::msg::FormationControlAgentData& msg);

    // To subscribe to the B2 agent output
    bool task_status_agent_2_;
    bool controller_status_agent_2_;
    double error_agent_2_;
    std::string status_msg_agent_2_;
    DQ xce2_;
    DQ yce2_;

    //bool new_B1Z1_agent_2_output_available_{false};
    //Subscription<consensus_protocol_msgs::msg::FormationControlAgentData>::SharedPtr subscriber_external_B1Z1_agent_data_2;
    //void _callback_external_B1Z1_agent_2_output(const consensus_protocol_msgs::msg::FormationControlAgentData& msg);


    // To subscribe to the pause status
    bool pause_switch_status_;
    bool pause_taskviz_status_;
    bool pause_status_;

    Publisher<sas_msgs::msg::Bool>::SharedPtr publisher_pause_taskviz_status_;

    void _callback_pause_switch_status(const sas_msgs::msg::Bool& status);
    Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_pause_switch_status_;

    void _callback_pause_status(const sas_msgs::msg::Bool& status);
    Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_pause_status_;

    bool switch_hearbeat_received_;
    void _callback_switch_hearbeat(const std_msgs::msg::Header& msg);
    Subscription<std_msgs::msg::Header>::SharedPtr subscriber_switch_hearbeat_;


/*
    DQ agent_3_pose_{1};
    bool new_agent_3_pose_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_agent_3_pose_state_;
    //void _callback_agent_3_pose_state(const geometry_msgs::msg::PoseStamped& msg);


    DQ human_cylinder_pose_{1};
    bool new_human_cylinder_pose_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_human_cylinder_pose_state_;
    void _callback_human_cylinder_pose_state(const geometry_msgs::msg::PoseStamped& msg);
    */


    void _update_robot_state();
    //void _update_agent_state();
    void _update_pause_status();
    void _publish_taskviz_pause_status();

    bool emergency_stop_triggered_;
    //bool pause_triggered_;
};
