/**
 * @file SensorFusion.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef SENSOR_FUSION_NODE_HPP
#define SENSOR_FUSION_NODE_HPP

#include <common_utils/rotation_helpers.hpp>
#include <kalman_filter_core/EKFFilter.hpp>
#include <motion_model/baseMotionModel.hpp>
#include <motion_model/constantAccelrationModel.hpp>
#include <motion_model/constantAccelerationWorldModel.hpp>

#include <observation_model/baseObservationModel.hpp>
#include <observation_model/ImuOdomObservationModel.hpp>
#include <observation_model/ImuOdomBodyFrameObservationModel.hpp>
#include <kalman_filter_app/BaseSensorFusion.hpp>
#include <kalman_filter_app/SensorFusionImuOdom.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SensorFusionNode : public rclcpp::Node
{

public:
    explicit SensorFusionNode(const std::string& name);
    ~SensorFusionNode() = default;

    void imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg);
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> imu_msg);

private:
    void publish_estimate(const builtin_interfaces::msg::Time &stamp);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<sensor_fusion::SensorFusionImuOdom> filter;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::tuple<double, double, double> current_pose2d;
    std::tuple<double, double, double> initial_pose2d;
    bool initial_pose_flag = false;
    bool initial_imu_flag = false;
    bool imu_received_after_init = false;

    // model_fixes=false keeps the original model (version A); true uses the world-frame
    // motion model + body-frame observation model (version B).
    bool model_fixes = false;
    // Only used with model_fixes: feed the odom pose (x, y, theta) as a measurement, or only its twist.
    bool use_odom_pose = true;

    double initial_imu_thetha = 0.0, normalize_imu_delta_angle = 0.0;
    double normalize_imu_angle = 0.0;
    double ax = 0.0, ay = 0.0, imu_w = 0.0;
    double last_odom_time = 0.0;

    Eigen::VectorXd Mu;
    Eigen::MatrixXd Cov;
    Eigen::MatrixXd Q_process_noise;
    Eigen::MatrixXd R_observation_state;

    Eigen::VectorXd u_control;
    Eigen::VectorXd z_observation;
    std::shared_ptr<motionmodel::BaseMotionModel> motion_model_;
    std::shared_ptr<observationmodel::BaseObservationModel> observation_model_;

}; //

#endif
