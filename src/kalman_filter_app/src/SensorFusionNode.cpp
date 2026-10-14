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

    this->declare_parameter<bool>("model_fixes", false);
    this->declare_parameter<bool>("use_odom_pose", true);

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

    model_fixes = this->get_parameter("model_fixes").as_bool();
    use_odom_pose = this->get_parameter("use_odom_pose").as_bool();

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

    
    std::vector<int> angle_indices;
    int expected_dim_z;
    if (model_fixes)
    {
        motion_model_ = std::make_shared<motionmodel::ConstantAccWorldMotionModel>();
        auto body_frame_model = std::make_shared<observationmodel::ImuOdomBodyFrameObservationModel>(use_odom_pose);
        angle_indices = body_frame_model->angle_indices;
        expected_dim_z = body_frame_model->dim_z;
        observation_model_ = body_frame_model;
    }
    else
    {
        motion_model_ = std::make_shared<motionmodel::ConstantAccMotionModel>();
        observation_model_ = std::make_shared<observationmodel::ImuOdomObservationModel>();
        angle_indices = {2, 3};
        expected_dim_z = 7;
    }

    if (Q_process_noise.rows() != 8 || Q_process_noise.cols() != 8 || Cov.rows() != 8 || Mu.size() != 8 ||
        R_observation_state.rows() != expected_dim_z || R_observation_state.cols() != expected_dim_z)
    {
        throw std::runtime_error("Q/Cov must be 8x8, X_init size 8 and R " + std::to_string(expected_dim_z) + "x" +
                                 std::to_string(expected_dim_z) + " for this configuration");
    }

    filter = std::make_shared<sensor_fusion::SensorFusionImuOdom>(
        Mu, Cov, Q_process_noise, R_observation_state, motion_model_, observation_model_);
    filter->ekf_filter_->setAngleIndices(angle_indices);
    // The original implementation linearized at the predicted mean; keep that behaviour for version A.
    filter->ekf_filter_->setLinearizeAtPreviousMean(model_fixes);

    RCLCPP_INFO(this->get_logger(), "model_fixes=%d use_odom_pose=%d dim_z=%d", model_fixes, use_odom_pose, expected_dim_z);

    fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odom", 50);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 50, std::bind(&SensorFusionNode::odom_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 50, std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));
}

void SensorFusionNode::imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg)
{
    ax = imu_msg->linear_acceleration.x;
    ay = imu_msg->linear_acceleration.y;
    imu_w = imu_msg->angular_velocity.z;

    if (model_fixes)
    {
        // Remove gravity: rotate the world gravity vector into the IMU frame using roll/pitch/yaw from the IMU.
        tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
        tf2::Vector3 g_body = tf2::Matrix3x3(q).transpose() * tf2::Vector3(0.0, 0.0, 9.80665);
        ax -= g_body.x();
        ay -= g_body.y();
    }

    double imu_thetha = quat_to_yaw(imu_msg->orientation);

    if (!initial_imu_flag)
    {
        initial_imu_thetha = imu_thetha;
        initial_imu_flag = true;
        return;
    }

    double delta_thetha = normalize_angle(imu_thetha - initial_imu_thetha);
    normalize_imu_delta_angle = delta_thetha;
    imu_received_after_init = true;
}

void SensorFusionNode::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg)
{
    double t = rclcpp::Time(odom_msg->header.stamp).seconds();

    if (!initial_pose_flag)
    {
        if (model_fixes)
            initial_pose2d = odom_to_pose2D(odom_msg);
        else
            initial_pose2d = rotate_pose2D(odom_to_pose2D(odom_msg), -90);
        initial_pose_flag = true;
        last_odom_time = t;
        return;
    }
    // Wait until the IMU reference yaw exists, otherwise theta_imu is meaningless.
    if (!imu_received_after_init)
    {
        last_odom_time = t;
        return;
    }

    double dt = t - last_odom_time;
    last_odom_time = t;
    if (dt <= 0.0)
        return;

    if (model_fixes)
    {
        // Express the odom pose in the initial body frame so it shares a reference with the relative IMU yaw.
        auto [x, y, yaw] = odom_to_pose2D(odom_msg);
        auto [x0, y0, yaw0] = initial_pose2d;
        double c = std::cos(yaw0), s = std::sin(yaw0);
        current_pose2d = {c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0), normalize_angle(yaw - yaw0)};
    }
    else
    {
        current_pose2d = get_normalize_pose2D(initial_pose2d, rotate_pose2D(odom_to_pose2D(odom_msg), -90));
    }

    std::vector<double> u_control_vec = {odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.angular.z};
    u_control = Eigen::Map<Eigen::VectorXd>(u_control_vec.data(), u_control_vec.size());

    std::tie(Mu, Cov) = filter->ekf_filter_->predict(u_control, dt);

    std::vector<double> z_vec;
    double v_fwd = odom_msg->twist.twist.linear.x;
    double v_lat = odom_msg->twist.twist.linear.y;
    if (!model_fixes)
        z_vec = {std::get<0>(current_pose2d), std::get<1>(current_pose2d), std::get<2>(current_pose2d), normalize_imu_delta_angle, imu_w, ax, ay};
    else if (use_odom_pose)
        z_vec = {std::get<0>(current_pose2d), std::get<1>(current_pose2d), std::get<2>(current_pose2d), normalize_imu_delta_angle, imu_w, v_fwd, v_lat, ax, ay};
    else
        z_vec = {normalize_imu_delta_angle, imu_w, v_fwd, v_lat, ax, ay};

    z_observation = Eigen::Map<Eigen::VectorXd>(z_vec.data(), z_vec.size());
    std::tie(Mu, Cov) = filter->ekf_filter_->update(z_observation, dt);

    publish_estimate(odom_msg->header.stamp);
}

void SensorFusionNode::publish_estimate(const builtin_interfaces::msg::Time &stamp)
{
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = "ekf_odom";
    out.child_frame_id = "base_link";

    out.pose.pose.position.x = Mu(0);
    out.pose.pose.position.y = Mu(1);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, normalize_angle(Mu(2)));
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();

    // Pose covariance (row-major 6x6 over x, y, z, roll, pitch, yaw): fill the x, y, yaw block.
    const int idx[3] = {0, 1, 5};
    const int st[3] = {0, 1, 2};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out.pose.covariance[idx[i] * 6 + idx[j]] = Cov(st[i], st[j]);

    // Twist is reported in the world (ekf_odom) frame, as estimated.
    out.twist.twist.linear.x = Mu(3);
    out.twist.twist.linear.y = Mu(4);
    out.twist.twist.angular.z = Mu(5);

    fused_odom_pub_->publish(out);
}
