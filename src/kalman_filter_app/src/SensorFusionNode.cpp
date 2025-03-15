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

#include <kalman_filter_app/SensorFusionNode.hpp>

SensorFusionNode::SensorFusionNode(const std::string &name) : rclcpp::Node(name)
{
    // Declare ROS2 Parameters
    this->declare_parameter<std::vector<double>>("Q_process_noise", std::vector<double>({0.0}));
    this->declare_parameter<int>("Q_rows", 1);
    this->declare_parameter<int>("Q_cols", 1);

    this->declare_parameter<std::vector<double>>("R_observation_noise", std::vector<double>({0.0}));
    this->declare_parameter<int>("R_rows", 1);
    this->declare_parameter<int>("R_cols", 1);

    this->declare_parameter<std::vector<double>>("X_init", std::vector<double>({0.0}));
    this->declare_parameter<int>("X_init_size", 1);

    this->declare_parameter<std::vector<double>>("Cov_init", std::vector<double>({0.0}));
    this->declare_parameter<int>("Cov_init_rows", 1);
    this->declare_parameter<int>("Cov_init_cols", 1);

    // Load parameters from ROS2
    std::vector<double> Q_data = this->get_parameter("Q_process_noise").as_double_array();
    int Q_rows = this->get_parameter("Q_rows").as_int();
    int Q_cols = this->get_parameter("Q_cols").as_int();

    std::vector<double> R_data = this->get_parameter("R_observation_noise").as_double_array();
    int R_rows = this->get_parameter("R_rows").as_int();
    int R_cols = this->get_parameter("R_cols").as_int();

    std::vector<double> X_data = this->get_parameter("X_init").as_double_array();
    int X_rows = this->get_parameter("X_init_size").as_int();

    std::vector<double> Cov_data = this->get_parameter("Cov_init").as_double_array();
    int Cov_rows = this->get_parameter("Cov_init_rows").as_int();
    int Cov_cols = this->get_parameter("Cov_init_cols").as_int();

    // Debug print statements
    std::cout << "Q_process_noise (size: " << Q_data.size() << "): ";
    for (double val : Q_data)
        std::cout << val << " ";
    std::cout << "\nQ_rows: " << Q_rows << ", Q_cols: " << Q_cols << std::endl;

    std::cout << "R_observation_noise (size: " << R_data.size() << "): ";
    for (double val : R_data)
        std::cout << val << " ";
    std::cout << "\nR_rows: " << R_rows << ", R_cols: " << R_cols << std::endl;

    std::cout << "X_init (size: " << X_data.size() << "): ";
    for (double val : X_data)
        std::cout << val << " ";
    std::cout << "\nX_rows: " << X_rows << std::endl;

    std::cout << "Cov_init (size: " << Cov_data.size() << "): ";
    for (double val : Cov_data)
        std::cout << val << " ";
    std::cout << "\nCov_rows: " << Cov_rows << ", Cov_cols: " << Cov_cols << std::endl;
    

    
    Q_process_noise = Eigen::Map<Eigen::MatrixXd>(Q_data.data(), Q_rows, Q_cols);
    R_observation_state = Eigen::Map<Eigen::MatrixXd>(R_data.data(), R_rows, R_cols);
    Mu = Eigen::Map<Eigen::VectorXd>(X_data.data(), X_rows);
    Cov = Eigen::Map<Eigen::MatrixXd>(Cov_data.data(), Cov_rows, Cov_cols);

    
    motion_model_ = std::make_shared<motionmodel::ConstantAccMotionModel>();
    observation_model_ = std::make_shared<observationmodel::ImuOdomObservationModel>();

    
    filter = std::make_shared<sensor_fusion::SensorFusionImuOdom>(
        Mu, Cov, Q_process_noise, R_observation_state, motion_model_, observation_model_);

    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&SensorFusionNode::odom_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));
}

void SensorFusionNode::imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg)
{
    ax = imu_msg->linear_acceleration.x;
    ay = imu_msg->linear_acceleration.x;
    imu_w = imu_msg->angular_velocity.z;

    double imu_thetha = quat_to_yaw(imu_msg->orientation);

    if (!initial_imu_flag)
    {
        initial_imu_thetha = imu_thetha;
        initial_imu_flag = true;
        return;
    }

    double delta_thetha = normalize_angle(imu_thetha - initial_imu_thetha);
    normalize_imu_delta_angle = delta_thetha;
}

void SensorFusionNode::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg)
{

    if (!initial_pose_flag)
    {

        initial_pose2d = rotate_pose2D(odom_to_pose2D(odom_msg), -90);
        initial_pose_flag = true;
        return;
    }
    current_pose2d = get_normalize_pose2D(initial_pose2d, rotate_pose2D(odom_to_pose2D(odom_msg), -90));

    std::vector<double> u_control_vec = {odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.angular.z};
    double dt = 0.1; // make thi based on two msg time diff
    u_control = z_observation = Eigen::Map<Eigen::VectorXd>(u_control_vec.data(), u_control_vec.size());

    std::tie(Mu, Cov) = filter->ekf_filter_->predict(u_control, dt);
    std::vector<double> z_vec = {std::get<0>(current_pose2d), std::get<1>(current_pose2d), std::get<2>(current_pose2d), normalize_imu_delta_angle, imu_w, ax, ay};
    std::cout << " MU " << " Cov " << std::endl;

    z_observation = Eigen::Map<Eigen::VectorXd>(z_vec.data(), z_vec.size());
    std::tie(Mu, Cov) = filter->ekf_filter_->update(z_observation, dt);
    std::cout << " MU " << " Cov" << std::endl;
}
