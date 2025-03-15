/**
 * @file SensorFusion.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <SensorFusionNode.hpp>


SensorFusionNode::SensorFusionNode(const string name) : Node(name)
{
    this->declare_parameter<std::vector<double>>("Q_process_noise");
    this->declare_parameter<int>("Q_rows");
    this->declare_parameter<int>("Q_cols");

    this->declare_parameter<std::vector<double>>("R_observation_noise");
    this->declare_parameter<int>("R_rows");
    this->declare_parameter<int>("R_cols");

    this->declare_parameter<std::vector<double>>("X_init");
    this->declare_parameter<int>("X_init_rows");
    this->declare_parameter<int>("X_init_cols");

    this->declare_parameter<std::vector<double>>("Cov_init");
    this->declare_parameter<int>("Cov_init_rows");
    this->declare_parameter<int>("Cov_init_cols");

    Q_process_noise = Eigen::Map<Eigen::MatrixXd>(this->get_parameter("Q_process_noise").as_double_array(), this->get_parameter("Q_rows").as_int(), this->get_parameter("Q_cols").as_int());
    R_observation_state = Eigen::Map<Eigen::MatrixXd>(this->get_parameter("R_observation_noise").as_double_array(), this->get_parameter("R_rows").as_int(), this->get_parameter("R_cols").as_int());

    Mu = Eigen::VectorXd(this->get_parameter("X_init").as_double_array(), , this->get_parameter("X_init_rows").as_int(), this->get_parameter("X_init_cols").as_int());
    cov = Eigen::Map<Eigen::MatrixXd>(this->get_parameter("Cov_init").as_double_array(), this->get_parameter("Cov_init_rows").as_int(), this->get_parameter("Cov_init_cols").as_int());

    motionmodel_ = std::make_shared<motionmodel::ConstantAccMotionModel>();

    observation_model_ = std::make_shared<observationmodel::ImuOdomObservationModel>();

    filter = std::make_shared<sensor_fusion::SensorFusionImuOdom>(Mu, cov, Q_process_noise, R_observation_state, motion_model, observation_model);
    odom_sub_ = rclcpp::create_subscription<sensor_msgs::msgs::Odometry>("/odom", 10, std::bind(&SensorFusionNode::odom_callback, this, std::Placeholder::_1));
    imu_sub_ = rclcpp::create_subscription<sensor_msgs::msgs::Odometry>("/imu", 10, std::bind(&SensorFusionNode::imu_callback, this, std::Placeholder::_1))
}

void SensorFusionNode::imu_callback(const std::shared_ptr<nav_msgs::msgs::Imu> imu_msg)
{
    ax = imu_msg->linear_accelration.x;
    ay = imu_msg->linear_accelration.x;
    imu_w = imu_msg->angular_velocity.z;

    double imu_thetha = quat_to_yaw(imu_msg->orientation);

    if (!initial_imu_flag)
    {
        imu_initial_thetha = imu_thetha;
        initial_imu_flag = true;
        return;
    }

    delta_thetha = normalize_angle(imu_theta - imu_initial_thetha);
    normalize_imu_delta_angle = delta_thetha;
}

void SensorFusionNode::odom_callback(const std::shared_ptr<nav_msgs::msgs::Odometry> odom_msg)
{

    if (!initial_pose_flag)
    {

        initial_pose2d = rotate_pose2D(odom_to_pose2D(imu_msg.get()), -90);
        initial_pose_flag = true;
        return;
    }
    current_pose2d = get_normalized_pose2D(initial_pose2d, rotate_pose2D(odom_to_pose2D(imu_msg.get()), -90));

    u_control = {odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.angular} dt = 0.1 Mu, Cov = filter.predict(u, 0.1) mu, cov = self.kf.predict(self.u, dt);

    [ mu, cov ] = filter.predict(u_control, dt);
    z_observation = {std::get<0>(current_pose2d), std::get<1>(current_pose2d), std::get<2>(current_pose2d), normalize_imu_delta_angle, imu_w, imu_a_x, imu_a_y} mu,
    [ mu, cov ] = filter.update(z_observation, dt);
}
