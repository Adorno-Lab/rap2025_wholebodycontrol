#include "sas_b1z1_coppeliasim_ros.hpp"


#include <iostream>
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>
//using std::placeholders::_1;

namespace sas
{


B1Z1CoppeliaSimROS::B1Z1CoppeliaSimROS(std::shared_ptr<Node> &node,
                                       const RobotDriverB1Z1CoppeliaSimConfiguration &configuration,
                                       std::atomic_bool *break_loops)
    :st_break_loops_{break_loops},
    configuration_{configuration},
    node_{node}, print_count_{0},
    clock_{configuration.thread_sampling_time_sec}
{

    cs_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();

    z1_name_ = configuration_.cs_B1_robotname + "/UnitreeZ1";

    FR_hip_rotor_ = configuration_.cs_B1_robotname + "/FR_hip_rotor_joint";
    FL_hip_rotor_ = configuration_.cs_B1_robotname + "/FL_hip_rotor_joint";
    RR_hip_rotor_ = configuration_.cs_B1_robotname + "/RR_hip_rotor_joint";
    RL_hip_rotor_ = configuration_.cs_B1_robotname + "/RL_hip_rotor_joint";

    subscriber_FR_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        configuration_.B1_topic_prefix + "/get/FR_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_FR_joint_states, this, std::placeholders::_1)
        );
    subscriber_FL_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        configuration_.B1_topic_prefix + "/get/FL_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_FL_joint_states, this, std::placeholders::_1)
        );
    subscriber_RR_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        configuration_.B1_topic_prefix + "/get/RR_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_RR_joint_states, this, std::placeholders::_1)
        );
    subscriber_RL_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        configuration_.B1_topic_prefix + "/get/RL_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_RL_joint_states, this, std::placeholders::_1)
        );

    subscriber_pose_state_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix + "/get/ekf/robot_pose", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_pose_state, this, std::placeholders::_1)
        );

    subscriber_robot_marker_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix + "/get/ekf/robot_marker_pose", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_robot_marker, this, std::placeholders::_1)
        );

    subscriber_Z1_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        configuration_.Z1_topic_prefix + "/get/joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_Z1_joint_states, this, std::placeholders::_1)
        );

    subscriber_x_fkm_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix+ "/set/coppeliasim_frame_x", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_x_fkm_state, this, std::placeholders::_1)
        );

    publisher_coppeliasim_frame_xd_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix + "/get/coppeliasim_frame_xd", 1);



    publisher_gripper_position_from_coppeliasim_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "coppeliasim/get/gripper_position", 1);



    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


}

void B1Z1CoppeliaSimROS::_connect()
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
        auto jointnames_aux = cs_->get_jointnames_from_object(z1_name_);
        arm_jointnames_ = jointnames_aux;
        arm_jointnames_.pop_back();
        gripper_joint_name_ = jointnames_aux.back();

        FR_jointnames_ = cs_->get_jointnames_from_object(FR_hip_rotor_);
        FL_jointnames_ = cs_->get_jointnames_from_object(FL_hip_rotor_);
        RR_jointnames_ = cs_->get_jointnames_from_object(RR_hip_rotor_);
        RL_jointnames_ = cs_->get_jointnames_from_object(RL_hip_rotor_);

    }
    catch (std::exception& e)
    {
        std::cout<<e.what()<<std::endl;
    }
}

void B1Z1CoppeliaSimROS::_set_joint_states_on_coppeliasim()
{

    if (qFR_cmd_.size() == 3 and qFL_cmd_.size() == 3 and qRR_cmd_.size() == 3 and qRL_cmd_.size() == 3)
    {
        cs_->set_joint_positions(FR_jointnames_, qFR_cmd_);
        cs_->set_joint_positions(FL_jointnames_, qFL_cmd_);
        cs_->set_joint_positions(RR_jointnames_, qRR_cmd_);
        cs_->set_joint_positions(RL_jointnames_, qRL_cmd_);
    }
    if (q_arm_cmd_.size() != 0 )
        cs_->set_joint_positions(arm_jointnames_, q_arm_cmd_.head(6));
}

void B1Z1CoppeliaSimROS::_set_robot_pose_on_coppeliasim(const DQ &pose)
{
    try
    {
        if (is_unit(pose)) 
            cs_->set_object_pose(configuration_.cs_B1_robotname, pose);
        if (is_unit(x_fkm_))
            cs_->set_object_pose(configuration_.robot_model_frame_name, x_fkm_);
        if (is_unit(robot_marker_))
            cs_->set_object_pose(configuration_.robot_marker_frame_name, robot_marker_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), std::string(__FUNCTION__)+std::string("::")+ e.what());
    }

}

void B1Z1CoppeliaSimROS::_read_xd_and_publish()
{
    geometry_msgs::msg::PoseStamped ros_msg_frame_xd;
    DQ xd = cs_->get_object_pose(configuration_.desired_frame_name);


    VectorXd position = xd.translation().vec3();
    ros_msg_frame_xd.pose.position.x = position(0);
    ros_msg_frame_xd.pose.position.y = position(1);
    ros_msg_frame_xd.pose.position.z = position(2);

    VectorXd orientation = xd.rotation().vec4();
    ros_msg_frame_xd.pose.orientation.w = orientation(0);
    ros_msg_frame_xd.pose.orientation.x = orientation(1);
    ros_msg_frame_xd.pose.orientation.y = orientation(2);
    ros_msg_frame_xd.pose.orientation.z = orientation(3);

    publisher_coppeliasim_frame_xd_->publish(ros_msg_frame_xd);
}


bool B1Z1CoppeliaSimROS::_should_shutdown() const
{
    return (*st_break_loops_);
}


void B1Z1CoppeliaSimROS::control_loop()
{
    try{

        clock_.init();
        _connect();

        while (!new_robot_pose_data_available_ && !_should_shutdown())
        {
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for ekf robot pose data");
        }


        for (int i=0;i<5;i++)
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::setting initial configurations...");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);

            _set_robot_pose_on_coppeliasim(robot_pose_);
            _set_joint_states_on_coppeliasim();
            clock_.update_and_sleep();
        }
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Done!");


        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Control loop running!");
        while(!_should_shutdown())
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            _set_joint_states_on_coppeliasim();
            _set_robot_pose_on_coppeliasim(robot_pose_);
            _read_xd_and_publish();
            _publish_gripper_position();

            for (auto& marker : vicon_markers_)
            {
                auto [marker_detected, marker_pose] = _try_get_vicon_marker(marker);
                if (marker_detected)
                    cs_->set_object_pose(marker, marker_pose);

            }

            rclcpp::spin_some(node_);
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), std::string(__FUNCTION__)+std::string("::")+ e.what());
    }
}

void B1Z1CoppeliaSimROS::_callback_FR_joint_states(const sensor_msgs::msg::JointState &msg)
{
    qFR_cmd_ = std_vector_double_to_vectorxd(msg.position);
}

void B1Z1CoppeliaSimROS::_callback_FL_joint_states(const sensor_msgs::msg::JointState &msg)
{
    qFL_cmd_ = std_vector_double_to_vectorxd(msg.position);
}

void B1Z1CoppeliaSimROS::_callback_RR_joint_states(const sensor_msgs::msg::JointState &msg)
{
    qRR_cmd_ = std_vector_double_to_vectorxd(msg.position);
}

void B1Z1CoppeliaSimROS::_callback_RL_joint_states(const sensor_msgs::msg::JointState &msg)
{
    qRL_cmd_ = std_vector_double_to_vectorxd(msg.position);
}

void B1Z1CoppeliaSimROS::_callback_Z1_joint_states(const sensor_msgs::msg::JointState &msg)
{
    q_arm_cmd_ = std_vector_double_to_vectorxd(msg.position);
}

void B1Z1CoppeliaSimROS::_callback_pose_state(const geometry_msgs::msg::PoseStamped &msg)
{
    robot_pose_ =   geometry_msgs_pose_stamped_to_dq(msg);
    new_robot_pose_data_available_ = true;
}

void B1Z1CoppeliaSimROS::_callback_robot_marker(const geometry_msgs::msg::PoseStamped &msg)
{
    robot_marker_ = geometry_msgs_pose_stamped_to_dq(msg);
}

void B1Z1CoppeliaSimROS::_callback_x_fkm_state(const geometry_msgs::msg::PoseStamped &msg)
{
    x_fkm_ = geometry_msgs_pose_stamped_to_dq(msg).normalize();
}

DQ B1Z1CoppeliaSimROS::geometry_msgs_msg_TransformStamped_to_dq(const geometry_msgs::msg::TransformStamped &msg)
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


void B1Z1CoppeliaSimROS::_publish_gripper_position()
{
    std_msgs::msg::Float64MultiArray ros_msg;
    VectorXd q_gripper = cs_->get_joint_positions({gripper_joint_name_});
    gripper_position_ = q_gripper(0);

    std::vector<double> gripper_position = {gripper_position_};
    ros_msg.data = gripper_position;
    publisher_gripper_position_from_coppeliasim_->publish(ros_msg);
}

std::tuple<bool, DQ> B1Z1CoppeliaSimROS::_try_get_vicon_marker(const std::string &marker_name)
{
    bool status = false;
    DQ marker_pose = DQ(1);
    try {
        geometry_msgs::msg::TransformStamped msg = tf_buffer_->lookupTransform("world",
                                                                               marker_name,
                                                                                 tf2::TimePointZero);
        status = true;
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
        marker_pose = (nr + 0.5*E_*t*nr).normalize();
        return {status, marker_pose};

    } catch (const tf2::TransformException & ex) {
        return {status, marker_pose};
    };

}


B1Z1CoppeliaSimROS::~B1Z1CoppeliaSimROS()
{

}


}
