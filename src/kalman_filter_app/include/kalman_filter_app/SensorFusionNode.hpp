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

#include <observation_model/baseObservationModel.hpp>
#include <observation_model/ImuOdomObservationModel.hpp>
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::shared_ptr<sensor_fusion::SensorFusionImuOdom> filter;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::tuple<double, double, double> current_pose2d;
    std::tuple<double, double, double> initial_pose2d;
    bool initial_pose_flag;
    bool initial_imu_flag;

    double initial_imu_thetha, normalize_imu_delta_angle;
    double normalize_imu_angle;
    double ax, ay, imu_w;

    Eigen::VectorXd Mu;
    Eigen::MatrixXd Cov;
    Eigen::VectorXd Q_process_noise;
    Eigen::VectorXd R_observation_state;

    Eigen::VectorXd u_control;
    Eigen::VectorXd z_observation;
    std::shared_ptr<motionmodel::BaseMotionModel> motion_model_;
    std::shared_ptr<observationmodel::BaseObservationModel> observation_model_;

}; //

#endif