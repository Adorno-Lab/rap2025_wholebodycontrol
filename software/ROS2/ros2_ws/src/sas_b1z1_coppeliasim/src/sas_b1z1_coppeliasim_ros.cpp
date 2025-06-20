#include "sas_b1z1_coppeliasim_ros.hpp"


#include <iostream>
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>
#include "RobotDriverCoppeliaSimUnitreeB1Z1.hpp"
//using std::placeholders::_1;

namespace sas
{

class B1Z1CoppeliaSimROS::Impl
{

public:
   std::shared_ptr<RobotDriverCoppeliaSimUnitreeB1Z1> b1z1_cs_driver_;
   Impl()
   {

   };



};

B1Z1CoppeliaSimROS::B1Z1CoppeliaSimROS(std::shared_ptr<Node> &node,
                                       const RobotDriverB1Z1CoppeliaSimConfiguration &configuration,
                                       std::atomic_bool *break_loops)
    :st_break_loops_{break_loops},
    topic_prefix_b1_{configuration.B1_topic_prefix},
    topic_prefix_z1_{configuration.Z1_topic_prefix},
    node_{node}, print_count_{0}, clock_{configuration.thread_sampling_time_sec}
{
    impl_ = std::make_unique<B1Z1CoppeliaSimROS::Impl>();

    impl_->b1z1_cs_driver_ = std::make_shared<RobotDriverCoppeliaSimUnitreeB1Z1>(break_loops,
                                                                                 configuration.cs_B1_robotname,
                                                                                 configuration.cs_Z1_robotname,
                                                                                 configuration.cs_host,
                                                                                 configuration.cs_port,
                                                                                 configuration.cs_TIMEOUT_IN_MILISECONDS);
    impl_->b1z1_cs_driver_->connect();
    RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(),impl_->b1z1_cs_driver_->get_status_message());
    impl_->b1z1_cs_driver_->initialize();
    RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(),impl_->b1z1_cs_driver_->get_status_message());


    subscriber_FR_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_prefix_b1_ + "/get/FR_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_FR_joint_states, this, std::placeholders::_1)
        );
    subscriber_FL_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_prefix_b1_ + "/get/FL_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_FL_joint_states, this, std::placeholders::_1)
        );
    subscriber_RR_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_prefix_b1_ + "/get/RR_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_RR_joint_states, this, std::placeholders::_1)
        );
    subscriber_RL_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_prefix_b1_ + "/get/RL_joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_RL_joint_states, this, std::placeholders::_1)
        );

    subscriber_pose_state_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
         topic_prefix_b1_ + "/get/pose_state", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_pose_state, this, std::placeholders::_1)
        );

    subscriber_Z1_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_prefix_z1_ + "/get/joint_states", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_Z1_joint_states, this, std::placeholders::_1)
        );

    subscriber_x_fkm_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_prefix_b1_ + "/set/coppeliasim_frame_x", 1, std::bind(&B1Z1CoppeliaSimROS::_callback_x_fkm_state, this, std::placeholders::_1)
        );

    publisher_coppeliasim_frame_xd_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_prefix_b1_+ + "/get/coppeliasim_frame_xd", 1);

    publisher_robot_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "coppeliasim/get/robot_pose", 1);

    publisher_gripper_position_from_coppeliasim_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "coppeliasim/get/gripper_position", 1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


}

void B1Z1CoppeliaSimROS::_set_joint_states_on_coppeliasim()
{

    if (qFR_cmd_.size() == 3 and qFL_cmd_.size() == 3 and qRR_cmd_.size() == 3 and qRL_cmd_.size() == 3)
        impl_->b1z1_cs_driver_->set_leg_positions({qFR_cmd_, qFL_cmd_, qRR_cmd_, qRL_cmd_});

    if (q_arm_cmd_.size() != 0 )
    {
        impl_->b1z1_cs_driver_->set_arm_positions(q_arm_cmd_.head(6));
        //impl_->b1z1_cs_driver_->set_gripper_position(q_arm_cmd_(6));
    }



}

void B1Z1CoppeliaSimROS::_set_robot_pose_on_coppeliasim(const DQ &pose)
{
    try
    {
        if (is_unit(pose) )
        {
            //DQ r = IMU_pose_;//*x_IMU_orientation_offset_.P();
            impl_->b1z1_cs_driver_->set_IMU_pose(pose);//x_world_1_average_*offset);
        }

        if (is_unit(x_fkm_))
            impl_->b1z1_cs_driver_->set_effector_frame_pose(x_fkm_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

}

void B1Z1CoppeliaSimROS::_read_xd_and_publish()
{
    geometry_msgs::msg::PoseStamped ros_msg_frame_xd;
    DQ xd = impl_->b1z1_cs_driver_->get_desired_frame_pose();


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
        impl_->b1z1_cs_driver_->connect();
        impl_->b1z1_cs_driver_->initialize();
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Updating vicon markers...");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Waiting stable frames...");




        for (int i=0;i<5000;i++)
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            _update_vicon_markers();
            _update_frames();
            _set_joint_states_on_coppeliasim();
        }
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Done!");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Updating vicon markers...");

        for (int i=0;i<1000;i++)
        {
            impl_->b1z1_cs_driver_->set_IMU_pose(IMU_pose_);
            clock_.update_and_sleep();
            _update_vicon_markers();
            _update_frames();
            _set_joint_states_on_coppeliasim();
            rclcpp::spin_some(node_);
        }

        for (int i=0;i<1000;i++)
        {
            DQ rimu = IMU_pose_.P();
            VectorXd pimu = IMU_pose_.translation().vec3();
            VectorXd p = x_world_1_average_.translation().vec3();
            robot_pose_ = rimu + 0.5*E_*DQ(0, p(0), p(1), pimu(2))*rimu;
            impl_->b1z1_cs_driver_->set_IMU_pose(robot_pose_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
        }
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Computing offsets...");

        for (int i=0;i<500;i++)
        {
            x_IMU_orientation_offset_ = IMU_pose_.conj()*x_world_1_average_;
            //UnitreeB1_initial_pose_ = impl_->b1z1_cs_driver_->get_IMU_pose();
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
        }


        robot_pose_ = _compute_robot_pose_from_IMU_and_markers();
        impl_->b1z1_cs_driver_->set_IMU_pose(robot_pose_);



        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Updating robot state...");

        for (int i=0;i<500;i++)
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            x_IMU_orientation_offset_ = IMU_pose_.conj()*x_world_1_average_;
            _update_vicon_markers();
            _update_frames();
            rclcpp::spin_some(node_);

        }



        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Done!");

        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Control loop running!");
        while(not _should_shutdown())
        {
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);

            _set_joint_states_on_coppeliasim();
            //_set_robot_pose_on_coppeliasim();
            robot_pose_ = _compute_robot_pose_from_IMU_and_markers();
            _set_robot_pose_on_coppeliasim(robot_pose_);
            _publish_robot_pose();

            _read_xd_and_publish();
            _update_vicon_markers();
            _publish_gripper_position();

            if (is_unit(x_world_1_average_ ))
                impl_->b1z1_cs_driver_->set_b1z1_frame_1(x_world_1_average_);



            rclcpp::spin_some(node_);
        }
        //impl_->unitree_b1_driver_->set_high_level_speed(0,0,0);
    }
    catch(const std::exception& e)
    {
        std::cerr<<"::Exception caught::" << e.what() <<std::endl;
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
    IMU_pose_ =   geometry_msgs_pose_stamped_to_dq(msg).normalize();
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

void B1Z1CoppeliaSimROS::_update_vicon_markers()
{

    try {

        std::vector<std::string> vicon_markers = {"B1Z1_Frame_0", "B1Z1_Frame_1", "B1Z1_Frame_2", "B1Z1_Frame_3", "B1Z1_Frame_4"};
        std::vector<geometry_msgs::msg::TransformStamped> TF;

        std::vector<DQ> dq_markers = {DQ(1), DQ(1), DQ(1), DQ(1), DQ(1)};

        for (size_t i=0;i<vicon_markers.size();i++)
        {
            DQ x;
            try {
                //TF.push_back(tf_buffer_->lookupTransform("world", vicon_markers.at(i),tf2::TimePointZero));
                x = geometry_msgs_msg_TransformStamped_to_dq(tf_buffer_->lookupTransform("world", vicon_markers.at(i),tf2::TimePointZero)).normalize();
            } catch (const tf2::TransformException & ex) {
                x = DQ(1);
                RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Marker "+vicon_markers.at(i)+" not detected!");
                //return;
            }
            dq_markers.at(i) = x;
        }

/*
        DQ x_world_0 = geometry_msgs_msg_TransformStamped_to_dq(TF.at(0)).normalize();
        DQ x_world_1 = geometry_msgs_msg_TransformStamped_to_dq(TF.at(1)).normalize();
        DQ x_world_2 = geometry_msgs_msg_TransformStamped_to_dq(TF.at(2)).normalize();
        DQ x_world_3 = geometry_msgs_msg_TransformStamped_to_dq(TF.at(3)).normalize();
        DQ x_world_4 = geometry_msgs_msg_TransformStamped_to_dq(TF.at(4)).normalize();
*/


        x_world_0_average_ = (x_world_0_previous_*DQ_robotics::pow((x_world_0_previous_.conj()* dq_markers.at(0)).normalize(), 0.1)).normalize();
        x_world_1_average_ = (x_world_1_previous_*DQ_robotics::pow((x_world_1_previous_.conj()* dq_markers.at(1)).normalize(), 0.1)).normalize();
        x_world_2_average_ = (x_world_2_previous_*DQ_robotics::pow((x_world_2_previous_.conj()* dq_markers.at(2)).normalize(), 0.1)).normalize();
        x_world_3_average_ = (x_world_3_previous_*DQ_robotics::pow((x_world_3_previous_.conj()* dq_markers.at(3)).normalize(), 0.1)).normalize();
        x_world_4_average_ = (x_world_4_previous_*DQ_robotics::pow((x_world_4_previous_.conj()* dq_markers.at(4)).normalize(), 0.1)).normalize();

        //RCLCPP_INFO_STREAM(node_->get_logger(), x_world_1_average_.vec8());
        //RCLCPP_INFO_STREAM(node_->get_logger(), DQ_robotics::pow(x_world_1_previous_.conj()*x_world_1, 0.01));

        x_world_0_previous_ = x_world_0_average_;
        x_world_1_previous_ = x_world_1_average_;
        x_world_2_previous_ = x_world_2_average_;
        x_world_3_previous_ = x_world_3_average_;
        x_world_4_previous_ = x_world_4_average_;




    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Problem in _update_vicon_markers()!!!");
        return;
    }


 //   RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), msg.child_frame_id.);
}



void B1Z1CoppeliaSimROS::_update_frames()
{
    if (is_unit(x_world_0_average_ ))
    {
        impl_->b1z1_cs_driver_->set_b1z1_frame_0(x_world_0_average_);
        //impl_->b1z1_cs_driver_->update_plane_0();
    }
    if (is_unit(x_world_1_average_ ))
        impl_->b1z1_cs_driver_->set_b1z1_frame_1(x_world_1_average_);
    if (is_unit(x_world_2_average_ ))
    {
        impl_->b1z1_cs_driver_->set_b1z1_frame_2(x_world_2_average_);
        //impl_->b1z1_cs_driver_->update_plane_2();
    }
    if (is_unit(x_world_3_average_ ))
    {
        impl_->b1z1_cs_driver_->set_b1z1_frame_3(x_world_3_average_);
        //impl_->b1z1_cs_driver_->update_plane_3();
    }
    if (is_unit(x_world_4_average_ ))
    {
        impl_->b1z1_cs_driver_->set_b1z1_frame_4(x_world_4_average_);
        //impl_->b1z1_cs_driver_->update_plane_4();
    }





}

DQ B1Z1CoppeliaSimROS::_compute_robot_pose_from_IMU_and_markers()
{
    DQ rimu = (IMU_pose_.P()*x_IMU_orientation_offset_.P()).normalize();
    VectorXd pimu = IMU_pose_.translation().vec3();
    VectorXd p = x_world_1_average_.translation().vec3();
    DQ x = rimu + 0.5*E_*DQ(0, p(0), p(1), pimu(2))*rimu;
    DQ offset = 1 + 0.5*E_*(0.27*i_);
    return (x*offset).normalize();

}

void B1Z1CoppeliaSimROS::_publish_robot_pose()
{
    geometry_msgs::msg::PoseStamped ros_msg_robot_pose;

    VectorXd position = robot_pose_.translation().vec3();
    ros_msg_robot_pose.pose.position.x = position(0);
    ros_msg_robot_pose.pose.position.y = position(1);
    ros_msg_robot_pose.pose.position.z = position(2);

    VectorXd orientation = robot_pose_.rotation().vec4();
    ros_msg_robot_pose.pose.orientation.w = orientation(0);
    ros_msg_robot_pose.pose.orientation.x = orientation(1);
    ros_msg_robot_pose.pose.orientation.y = orientation(2);
    ros_msg_robot_pose.pose.orientation.z = orientation(3);
    publisher_robot_pose_->publish(ros_msg_robot_pose);

}

void B1Z1CoppeliaSimROS::_publish_gripper_position()
{
    std_msgs::msg::Float64MultiArray ros_msg;
    gripper_position_ = impl_->b1z1_cs_driver_->get_gripper_position();
    std::vector<double> gripper_position = {gripper_position_};
    ros_msg.data = gripper_position;
    publisher_gripper_position_from_coppeliasim_->publish(ros_msg);
}




B1Z1CoppeliaSimROS::~B1Z1CoppeliaSimROS()
{
    impl_->b1z1_cs_driver_->deinitialize();
    impl_->b1z1_cs_driver_->disconnect();
}


}
