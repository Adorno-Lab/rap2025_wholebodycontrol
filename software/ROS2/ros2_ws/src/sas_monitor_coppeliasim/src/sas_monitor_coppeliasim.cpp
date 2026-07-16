
#include "sas_monitor_coppeliasim/sas_monitor_coppeliasim.hpp"
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>

namespace sas
{


MonitorCoppeliaSim::~MonitorCoppeliaSim()
{
    *st_break_loops_ = true;
}

MonitorCoppeliaSim::MonitorCoppeliaSim(std::shared_ptr<Node> &node,
                                                         const MonitorCoppeliaSimConfiguration &configuration,
                                                         std::atomic_bool *break_loops):
st_break_loops_{break_loops},
configuration_{configuration},
node_{node},
clock_{configuration.thread_sampling_time_sec}
{
    cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();

    robot_client_1_ = std::make_shared<UnitreeB1Z1RobotClient>(node_,
                                                             configuration_.B1_1_topic_prefix,
                                                             configuration_.Z1_1_topic_prefix,
                                                             UnitreeB1Z1RobotClient::MODE::MONITORING);



    subscriber_x1_fkm_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_1_topic_prefix+ "/set/coppeliasim_frame_x", 1,
        std::bind(&MonitorCoppeliaSim::_callback_x1_fkm_state, this, std::placeholders::_1)
        );


    publisher_x1d_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_1_topic_prefix+ "/get/coppeliasim_frame_xd",1
     );


    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());;
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    Z1_1_name_ = configuration_.cs_B1_1_robotname + "/UnitreeZ1";


}

void MonitorCoppeliaSim::control_loop()
{
    clock_.init();
    _connect();

    while(!_should_shutdown())
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Running control loop!");
        rclcpp::spin_some(node_);
        clock_.update_and_sleep();
        rclcpp::spin_some(node_);



        if (is_unit(x1_fkm_))
            _set_object_pose(configuration_.cs_B1Z1_1_frame_x, x1_fkm_);


        DQ xd = cs_->get_object_pose(configuration_.cs_B1Z1_1_frame_xd);
        if (is_unit(xd))
            publisher_x1d_->publish(sas::dq_to_geometry_msgs_pose_stamped(xd));


        if (robot_client_1_->is_enabled())
        {
            _set_joint_states_on_coppeliasim(Z1_1_arm_jointnames_, robot_client_1_->get_arm_joint_states_including_gripper());
            _set_joint_states_on_coppeliasim(B1_1_FR_jointnames_, robot_client_1_->get_leg_joint_states(UnitreeB1Z1RobotClient::LEG::FR));
            _set_joint_states_on_coppeliasim(B1_1_FL_jointnames_, robot_client_1_->get_leg_joint_states(UnitreeB1Z1RobotClient::LEG::FL));
            _set_joint_states_on_coppeliasim(B1_1_RL_jointnames_, robot_client_1_->get_leg_joint_states(UnitreeB1Z1RobotClient::LEG::RL));
            _set_joint_states_on_coppeliasim(B1_1_RR_jointnames_, robot_client_1_->get_leg_joint_states(UnitreeB1Z1RobotClient::LEG::RR));
            _set_object_pose(configuration_.cs_B1_1_robotname, robot_client_1_->get_b1_pose());
        }


        // read from CS the center of formation and publish on a ROS2 topic
       // _publish_pose_stamped_data_of_coppeliasim_object_object(publisher_center_of_formation_pose_state_,
       //                                                         configuration_.cs_center_of_formation_frame_x);
    }

}

void MonitorCoppeliaSim::_callback_x1_fkm_state(const geometry_msgs::msg::PoseStamped &msg)
{
    x1_fkm_ = geometry_msgs_pose_stamped_to_dq(msg);
}



std::optional<DQ> MonitorCoppeliaSim::_try_get_vicon_marker(const std::string& marker_name)
{
    try {
        geometry_msgs::msg::TransformStamped msg = tf_buffer_->lookupTransform("world",
                                                                               marker_name,
                                                                               tf2::TimePointZero);

        return _geometry_msgs_msg_TransformStamped_to_dq(msg);

    } catch (...) {  //const tf2::TransformException & ex
        RCLCPP_INFO_STREAM(node_->get_logger(), "Marker "+marker_name+" not detected!");
        return std::nullopt;
    };
}


DQ MonitorCoppeliaSim::_geometry_msgs_msg_TransformStamped_to_dq(const geometry_msgs::msg::TransformStamped &msg)
{
    const  DQ r  = DQ(msg.transform.rotation.w,
                    msg.transform.rotation.x,
                    msg.transform.rotation.y,
                    msg.transform.rotation.z);
    const  DQ nr = normalize(r);

    const DQ t(
        0,
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z);
    return (nr + 0.5*E_*t*nr).normalize();
}

void MonitorCoppeliaSim::_publish_pose_stamped_data(const Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
                                                        const DQ& pose)
{
    publisher->publish(sas::dq_to_geometry_msgs_pose_stamped(pose));
}

void MonitorCoppeliaSim::_publish_pose_stamped_data_of_coppeliasim_object_object(const Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& publisher,
                                                                                          const std::string &coppeliasim_object_name)
{
    try {
        publisher->publish(sas::dq_to_geometry_msgs_pose_stamped(cs_->get_object_pose(coppeliasim_object_name)));
    } catch (...) {
    }
}

void MonitorCoppeliaSim::_connect()
{
    try
    {
        if (!cs_->connect(configuration_.cs_host, configuration_.cs_port, configuration_.cs_TIMEOUT_IN_MILISECONDS))
        {
            throw std::runtime_error("Unable to connect to CoppeliaSim.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Connected to CoppeliaSim!");

        // Get the joint names from CoppeliaSim
        Z1_1_arm_jointnames_ = cs_->get_jointnames_from_object(Z1_1_name_);



        B1_1_FR_jointnames_ = {configuration_.cs_B1_1_robotname + "/FR_hip_joint",
                               configuration_.cs_B1_1_robotname + "/FR_thigh_joint",
                               configuration_.cs_B1_1_robotname + "/FR_calf_joint",
                               };

        B1_1_FL_jointnames_ = {configuration_.cs_B1_1_robotname + "/FL_hip_joint",
                               configuration_.cs_B1_1_robotname + "/FL_thigh_joint",
                               configuration_.cs_B1_1_robotname + "/FL_calf_joint",
                                };

        B1_1_RR_jointnames_ = {configuration_.cs_B1_1_robotname + "/RR_hip_joint",
                               configuration_.cs_B1_1_robotname + "/RR_thigh_joint",
                               configuration_.cs_B1_1_robotname + "/RR_calf_joint",
                               };

        B1_1_RL_jointnames_ = {configuration_.cs_B1_1_robotname + "/RL_hip_joint",
                               configuration_.cs_B1_1_robotname + "/RL_thigh_joint",
                               configuration_.cs_B1_1_robotname + "/RL_calf_joint",
                               };







    }
    catch (std::exception& e)
    {
        std::cout<<e.what()<<std::endl;
    }
}

bool MonitorCoppeliaSim::_should_shutdown() const
{
    return (*st_break_loops_);
}

std::tuple<double, double> MonitorCoppeliaSim::_get_switching_error(const DQ &x1, const DQ &x2)
{
    VectorXd error_1 =  vec8( x1.conj()*x2 - 1 );
    VectorXd error_2 =  vec8( x1.conj()*x2 + 1 );

    double norm_1 = error_1.norm();
    double norm_2 = error_2.norm();

    return {norm_1, norm_2};
}

void MonitorCoppeliaSim::_set_joint_states_on_coppeliasim(const std::vector<std::string> &jointnames,
                                                                   const VectorXd &q)
{
    if (q.size() > 0 && jointnames.size() == static_cast<size_t>(q.size()))
        cs_->set_joint_positions(jointnames, q);
    else
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "problem setting the joints ");
        RCLCPP_INFO_STREAM(node_->get_logger(), q.transpose());
        for (auto& name : jointnames)
            RCLCPP_INFO_STREAM(node_->get_logger(), name);
    }
}

void MonitorCoppeliaSim::_set_object_pose(const std::string &base_name, const DQ &pose)
{
    if (is_unit(pose))
        cs_->set_object_pose(base_name, pose);
}

}
