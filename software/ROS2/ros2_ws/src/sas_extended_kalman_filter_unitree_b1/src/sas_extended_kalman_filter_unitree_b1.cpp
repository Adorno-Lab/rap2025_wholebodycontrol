#include "sas_extended_kalman_filter_unitree_b1/sas_extended_kalman_filter_unitree_b1.hpp"

#include <memory>
#include <rclcpp/logging.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/utils/DQ_Geometry.h>


using namespace rclcpp;

namespace sas
{





/**
 * @brief ExtendedKalmanFilter::ExtendedKalmanFilter ctor of the class
 * @param node The ROS2 node
 * @param configuration the configuration parameters
 * @param break_loops flag to finish all loops
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(std::shared_ptr<rclcpp::Node> &node,
                                           const ExtendedKalmanFilterConfiguration &configuration,
                                           std::atomic_bool* break_loops)
:st_break_loops_{break_loops},configuration_{configuration},clock_{configuration.thread_sampling_time_sec},
    node_{node}, datalogger_client_{node}
{

    subscriber_IMU_state_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        configuration_.topic_prefix + "/get/IMU_state",
        1,
        std::bind(&ExtendedKalmanFilter::_callback_subscriber_IMU_state, this, std::placeholders::_1)
        );


    subscriber_twist_state_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        configuration_.topic_prefix + "/get/twist_state",
        1,
        std::bind(&ExtendedKalmanFilter::_callback_subscriber_twist_state, this, std::placeholders::_1)
        );


    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_estimated_robot_marker_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.topic_prefix +"/get/ekf/robot_marker_pose", 1);

    publisher_estimated_robot_marker_pose_with_offset_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.topic_prefix +"/get/ekf/robot_pose", 1);

    publisher_pose_debug_  = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.topic_prefix +"/get/ekf/debug_pose", 1);


    // Initialize state vector: [x, y, phi]
    x_.setZero(3);
    // x, y, phi, vx, vy, wz all initialized to 0

    // Initialize covariance matrix
    SIGMA_.setIdentity(3, 3);
    SIGMA_ *= 0.01; // Initial uncertainty

    // Process noise covariance
    R_.setIdentity(3, 3);
    R_ << 0.05,    0,        0,
             0, 0.05,        0,
             0,     0,  0.05;

    H_.setIdentity(3,3);
    // Measurement noise covariance for Vicon (x, y, phi)
    Q_.setIdentity(3, 3);
    Q_ << 1e-3,    0,        0,
             0, 1e-3,        0,
             0,     0,    1e-3;

}

/**
 * @brief ExtendedKalmanFilter::_should_shutdown return a boolean indicanting if the control loops are
 *                              requested to break.
 * @return True if the control loops are requested to break. False otherwise
 */
bool ExtendedKalmanFilter::_should_shutdown() const
{
    return (*st_break_loops_);
}

/**
 * @brief ExtendedKalmanFilter::_callback_subscriber_IMU_state callback method for the subscriber_IMU_state_
 * @param msg
 */
void ExtendedKalmanFilter::_callback_subscriber_IMU_state(const sensor_msgs::msg::Imu &msg)
{
    //const DQ w = msg.angular_velocity.x*i_ + msg.angular_velocity.y*j_ + msg.angular_velocity.z*k_;
    //const DQ p_dot_dot = msg.linear_acceleration.x*i_ + msg.linear_acceleration.y*j_ + msg.linear_acceleration.z*k_;
    const DQ r = sas::geometry_msgs_quaternion_to_dq(msg.orientation);

    // angular_velocity_IMU_ = w;
    // linear_acceleration_IMU_ = p_dot_dot;
    orientation_IMU_ = r;
    new_IMU_data_available_ = true;
}


/**
 * @brief ExtendedKalmanFilter::_callback_subscriber_twist_state callback method for the twist state suscriber
 * @param msg The TwistStamped message.
 */
void ExtendedKalmanFilter::_callback_subscriber_twist_state(const geometry_msgs::msg::TwistStamped &msg)
{
    angular_velocity_from_robot_data_ = msg.twist.angular.x*i_ + msg.twist.angular.y*j_ + msg.twist.angular.z*k_;
    linear_velocity_from_robot_data_  =  msg.twist.linear.x*i_  + msg.twist.linear.y*j_  + msg.twist.linear.z*k_;
    new_twist_data_available_ = true;
}




/**
 * @brief ExtendedKalmanFilter::control_loop starts the ROS 2 control loop
 */
void ExtendedKalmanFilter::control_loop()
{
    try{

        clock_.init();

        while (!new_IMU_data_available_ && !_should_shutdown())
        {
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for IMU data");

        }
        while (!new_twist_data_available_ && !_should_shutdown())
        {
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for Twist data");
        }
        while (!new_vicon_data_available_rear_ && !_should_shutdown())
        {
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            _try_update_vicon_markers();
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for rear VICON data");
        }
        while (!new_vicon_data_available_front_ && !_should_shutdown())
        {
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            _try_update_vicon_markers();
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for front VICON data");
        }

        const double K{100.0};

        DQ xk;
        DQ x_vicon_k_previous_= vicon_pose_;


        x_rear_to_front_ = vicon_pose_rear_.conj()*vicon_pose_front_;
        DQ x_rear_to_front_previous_= x_rear_to_front_;

        for (int i=0;i<K;i++)
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Initializing poses from Vicon...");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            xk = x_vicon_k_previous_*DQ_robotics::pow(x_vicon_k_previous_.conj()*vicon_pose_, 1/K);
            x_vicon_k_previous_ = xk;
            estimated_robot_pose_ = xk;


            DQ x_rear_to_front_current = vicon_pose_rear_.conj()*vicon_pose_front_;
            x_rear_to_front_ = x_rear_to_front_previous_*DQ_robotics::pow(x_rear_to_front_previous_.conj()*x_rear_to_front_current, 1/K);
            x_rear_to_front_previous_ = x_rear_to_front_;




            std::cout<<"Initializing the reference frames..."<<K-i<<std::endl;
            _try_update_vicon_markers();
            _publish_pose_stamped(publisher_estimated_robot_marker_pose_, "/world", estimated_robot_pose_);
            _publish_pose_stamped(publisher_estimated_robot_marker_pose_with_offset_, "/world", estimated_robot_pose_*x_offset_rear_to_central_body_frame_);
            _publish_pose_stamped(publisher_pose_debug_, "/world", vicon_pose_rear_*x_rear_to_front_);
            if (_should_shutdown())
                break;
        }
        VectorXd t = estimated_robot_pose_.translation().vec3();
        vicon_height_ = t(2);


        auto pose = _get_mobile_platform_configuration_from_pose(estimated_robot_pose_);
        auto phi = normalize_angle(pose(2));
        //x, y, phi, vx, vy, wz
        x_ << t(0), t(1), phi;


        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::IMU and VICON data available!");

        if (save_data_with_datalogger_)
        {

            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Waiting for a connection with sas_datalogger...");
            while( (!datalogger_client_.is_enabled()) && (!_should_shutdown()))
            {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                rclcpp::spin_some(node_);
            }
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "Connected to sas_datalogger!");
        }else{
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "sas_datalogger is disabled.");
        }


        while(!_should_shutdown())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Control loop running!");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            _try_update_vicon_markers();

            _prediction_step();

            if (new_vicon_data_available_rear_ || new_vicon_data_available_front_)
            {
                _update_step();
                RCLCPP_INFO_ONCE(node_->get_logger(), "::VICON DATA OK");
            }else{
                if (!new_vicon_data_available_rear_ )
                    RCLCPP_INFO_ONCE(node_->get_logger(), "::REAR MARKER LOST!!!");
                if (!new_vicon_data_available_front_)
                    RCLCPP_INFO_ONCE(node_->get_logger(), "::FRONT MARKER LOST!!!");
            }


            _publish_pose_stamped(publisher_estimated_robot_marker_pose_, "/world", estimated_robot_pose_);
            _publish_pose_stamped(publisher_estimated_robot_marker_pose_with_offset_, "/world", estimated_robot_pose_*x_offset_rear_to_central_body_frame_);
            _publish_pose_stamped(publisher_pose_debug_, "/world", vicon_pose_rear_*x_rear_to_front_);


            if (save_data_with_datalogger_)
            {
                VectorXd vicon_pose_vec = vicon_pose_.vec8();
                datalogger_client_.log("x_vicon", vicon_pose_vec);
                ///datalogger_client_.log("SIGMA", vicon_pose_vec);
            }
            rclcpp::spin_some(node_);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"::Exception caught::" << e.what() <<std::endl;
    }
}

/**
 * @brief ExtendedKalmanFilter::_try_update_vicon_markers update the Vicon data if available.
 *
 */
void ExtendedKalmanFilter::_try_update_vicon_markers()
{
    // The vicon_pose_ represents the measurement of the position and orientation of the B1 robot.
    // There are two markers rigidly attached to the robot. vicon_marker_rear_ and vicon_marker_front_.
    // If only the rear marker is detected, vicon_pose_ is vicon_marker_rear_.
    // If only the front marker is detected, we compute vicon_marker_rear_ using the rigid body transformation
    // x_rear_to_front_.
    // Finally, if both markers are detected, we compute the average bewteen them.
    try {
        geometry_msgs::msg::TransformStamped msg = tf_buffer_->lookupTransform("world",
                                                                               configuration_.robot_vicon_marker_rear,
                                                                               tf2::TimePointZero);
        const unsigned int stamp_nanosec = msg.header.stamp.nanosec;
        // Check if the stamp is different. Otherwise, the data is not new.
        if (stamp_nanosec != vicon_stamp_nano_rear_)
        {
            // Ok, the data is new. But, we need to check if the data is OK.
            // Sometimes, when the marker is near to the borders of the Vicon workspace,
            // the pose is completly wrong. Since we are modeling the B1 robot as a planar joint,
            // we assume the z-axis is perpendicular to the ground. The distance between the orientation
            // line attached to the z-axis of the marker, and the static line k_ must be lower than a threshold,
            // otherwise the measurement is rejected.

            DQ x_vicon_rear = _geometry_msgs_msg_TransformStamped_to_dq(msg);
            const double f = _compute_line_to_line_angle_distance(x_vicon_rear);

            if (f < 0.01)
            {
                vicon_stamp_nano_rear_ = stamp_nanosec;
                new_vicon_data_available_rear_ = true;
                vicon_pose_rear_ = x_vicon_rear;
                data_loss_counter_rear_= 0;

            }else
            {
                new_vicon_data_available_rear_ = false;
                //std::cerr<<"Vicon data rejected! error norm: "<<f<<std::endl;
            }

        }else{
            data_loss_counter_rear_++;
            if (data_loss_counter_rear_ > DATA_LOSS_THRESHOLD_)
                new_vicon_data_available_rear_ = false;
        }
    } catch (const tf2::TransformException & ex) {
        new_vicon_data_available_rear_ = false;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Marker "+configuration_.robot_vicon_marker_rear+" not detected!");
    };


    //-------------------------------------------------------------------------------------------------------------------
    try {
        geometry_msgs::msg::TransformStamped msg = tf_buffer_->lookupTransform("world",
                                                                               configuration_.robot_vicon_marker_front,
                                                                               tf2::TimePointZero);
        const unsigned int stamp_nanosec = msg.header.stamp.nanosec;
        // Check if the stamp is different. Otherwise, the data is not new.
        if (stamp_nanosec != vicon_stamp_nano_front_)
        {
            // Ok, the data is new. But, we need to check if the data is OK.
            // Sometimes, when the marker is near to the borders of the Vicon workspace,
            // the pose is completly wrong.

            DQ x_vicon_front = _geometry_msgs_msg_TransformStamped_to_dq(msg);
            const double f = _compute_line_to_line_angle_distance(x_vicon_front);
            // if the distance between the line orienations is above a threshold, the data is
            // rejected.
            if (f < 0.01)
            {
                vicon_stamp_nano_front_ = stamp_nanosec;
                new_vicon_data_available_front_ = true;
                vicon_pose_front_ = x_vicon_front;
                data_loss_counter_front_= 0;
            }else
            {
                new_vicon_data_available_front_ = false;
            }

        }else{
            data_loss_counter_front_++;
            if (data_loss_counter_front_ > DATA_LOSS_THRESHOLD_)
                new_vicon_data_available_front_ = false;
        }
    } catch (const tf2::TransformException & ex) {
        new_vicon_data_available_front_ = false;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Marker "+configuration_.robot_vicon_marker_front+" not detected!");
    };
    //--------------------------------------------------------------------------------------------------------------------
    // If only the rear marker is available, the vicon pose marker is set
    if (new_vicon_data_available_rear_ && !new_vicon_data_available_front_)
    {
        vicon_pose_ = vicon_pose_rear_;
        //updated the height
        _set_height_from_marker(vicon_pose_);
    }

    // If both are available, we compute the average
    if (new_vicon_data_available_rear_ && new_vicon_data_available_front_)
    {
        const DQ& x1 = vicon_pose_rear_;
        const DQ& x2 = vicon_pose_front_*x_rear_to_front_.conj();
        vicon_pose_  = x1*DQ_robotics::pow(x1.conj()*x2, 0.5);
        _set_height_from_marker(vicon_pose_);
    }

    // If the rear marker is lost, we compute its pose using the from marker and its corresponding offset.
    if (!new_vicon_data_available_rear_ && new_vicon_data_available_front_)
    {
        vicon_pose_ = vicon_pose_front_*x_rear_to_front_.conj();
        //updated the height
        _set_height_from_marker(vicon_pose_);
    }
}

/**
 * @brief ExtendedKalmanFilter::_geometry_msgs_msg_TransformStamped_to_dq
 * @param msg
 * @return
 */
DQ ExtendedKalmanFilter::_geometry_msgs_msg_TransformStamped_to_dq(const geometry_msgs::msg::TransformStamped &msg)
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

/**
 * @brief ExtendedKalmanFilter::_compute_line_to_line_angle_distance compute the distance function between
 *                          the orientation line attached to the z-axis of the vicon marker and the static line k_
 *                          This distance function corresponds to the equation (9) of [1].
 *
 *  [1] J. J. Quiroz-Omaña and B. V. Adorno,
 *  "Whole-Body Control With (Self) Collision Avoidance Using Vector Field Inequalities,"
 *  in IEEE Robotics and Automation Letters, vol. 4, no. 4, pp. 4048-4053,
 *  Oct. 2019, doi: 10.1109/LRA.2019.2928783.
 *
 * @param vicon_pose_marker
 * @return the desired distance between the orientation lines
 */
double ExtendedKalmanFilter::_compute_line_to_line_angle_distance(const DQ &vicon_pose_marker)
{
    const DQ workspace_line = k_;
    const DQ robot_line =  vicon_pose_marker.P()*(k_)*vicon_pose_marker.P().conj();
    const double phi = DQ_Geometry::line_to_line_angle(robot_line, workspace_line);
    const double f = 2-2*cos(phi);
    return f;
}

/**
 * @brief ExtendedKalmanFilter::_set_height_from_marker updates the height of the estimation pose
 * @param x pose The unit dual quaternion that represents the pose. This is usual a vicon marker pose.
 */
void ExtendedKalmanFilter::_set_height_from_marker(const DQ &x)
{
    VectorXd t = x.translation().vec3();
    vicon_height_ = t(2);
}


/**
 * @brief ExtendedKalmanFilter::_publish_pose_stamped publishes a ROS 2 Pose Stamped message
 * @param publisher The publisher pointer
 * @param frame_id The reference frame
 * @param pose The unit dual quaternion that represents the pose
 */
void ExtendedKalmanFilter::_publish_pose_stamped(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &publisher,
                                                 const std::string &frame_id,
                                                 const DQ &pose)
{
    geometry_msgs::msg::PoseStamped posed_stamped;
    posed_stamped.header = std_msgs::msg::Header();
    posed_stamped.header.frame_id = frame_id;
    posed_stamped.header.stamp = rclcpp::Clock().now();
    posed_stamped.pose = dq_to_geometry_msgs_pose(pose);
    publisher->publish(posed_stamped);
}

/**
 * @brief ExtendedKalmanFilter::_prediction_step performs the prediction based on the model
 */
void ExtendedKalmanFilter::_prediction_step()
{
    new_twist_data_available_ = false;
    double x = x_(0);
    double y = x_(1);
    double phi = x_(2);

    double dt = configuration_.thread_sampling_time_sec;

    const DQ& r_ = estimated_robot_pose_.P();

    // Express the velocities in the world frame
    DQ angular_velocity_world_frame = r_*angular_velocity_from_robot_data_*r_.conj();
    DQ linear_velocity_world_frame = r_*linear_velocity_from_robot_data_*r_.conj();

    VectorXd linear_velocities = linear_velocity_world_frame.vec3();
    VectorXd angular_velocities = angular_velocity_world_frame.vec3();
    const double& vx = linear_velocities(0);
    const double& vy = linear_velocities(1);
    const double& wz =  angular_velocities(2);

    x = x + vx * dt;
    y = y + vy * dt;

    phi += wz * dt;
    phi = normalize_angle(phi);

    // Update state vector
    x_(0) = x;
    x_(1) = y;
    x_(2) = phi;


    DQ r = cos(phi/2) + k_*sin(phi/2);
    DQ t = x*i_ + y*j_ +vicon_height_*k_;
    estimated_robot_pose_ = r + 0.5*E_*t*r;

    predicted_robot_pose_ = r + 0.5*E_*t*r;

    // Compute Jacobian for state transition
    double v = std::sqrt( vx*vx + vy*vy );
    Eigen::MatrixXd G = jacobian_matrix(v, phi);

    // Propagate covariance:
    SIGMA_ = G * SIGMA_ * G.transpose() + R_;
}

/**
 * @brief ExtendedKalmanFilter::_update_step performs the estimation update based on the measurements
 */
void ExtendedKalmanFilter::_update_step()
{
    Eigen::MatrixXd Kt = SIGMA_*H_.transpose()*(H_*SIGMA_*H_.transpose() + Q_).inverse();

    VectorXd z = _get_mobile_platform_configuration_from_pose(vicon_pose_);
    z(2) = normalize_angle(z(2));
    VectorXd h = H_*x_;

    x_ = x_ + Kt*(z-h);
    SIGMA_ = (Eigen::MatrixXd::Identity(3,3)-Kt*H_)*SIGMA_;

    const double& x = x_(0);
    const double& y = x_(1);
    const double& phi = x_(2);
    DQ r = cos(phi/2) + k_*sin(phi/2);
    DQ t = x*i_ + y*j_ +vicon_height_*k_;
    estimated_robot_pose_ = r + 0.5*E_*t*r;
}


/**
 * @brief ExtendedKalmanFilter::normalize_angle
 * @param angle
 * @return
 */
double ExtendedKalmanFilter::normalize_angle(double angle) {
    // Normalize angle to [-pi, pi]
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief ExtendedKalmanFilter::_get_mobile_platform_configuration_from_pose returns the configuration vector
 *                          q = [x,y,phi] given a unit dual quaternion
 * @param pose
 * @return The desired configuration vector
 */
VectorXd ExtendedKalmanFilter::_get_mobile_platform_configuration_from_pose(const DQ &pose) const
{
    DQ x = pose;
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}

/**
 * @brief ExtendedKalmanFilter::jacobian_matrix compute the Jacobian matrix of the model
 * @param v the linear velocitiy
 * @param phi the rotation angle
 * @return The deisred Jacobian matrix
 */
MatrixXd ExtendedKalmanFilter::jacobian_matrix(const double &v, const double &phi)
{
    const double& T = configuration_.thread_sampling_time_sec;
    Eigen::MatrixXd J(3, 3);
    J << 1, 0, -T*v*sin(phi),
         0, 0,  T*v*cos(phi),
         0, 0,             1;
    return J;
}


/**
 * @brief ExtendedKalmanFilter::~ExtendedKalmanFilter
 */
ExtendedKalmanFilter::~ExtendedKalmanFilter()
{

}

}
