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

#include <rclcpp/rclcpp>
#include <kalman_filter_core/EKFFilter.hpp>
#include <motion_model/baseMotionModel.hpp>
#include <observation_model/baseObservationModel.hpp>
#include <kalman_filter_app/BaseSensorFusion.hpp>

#include <rclcpp/rclcpp.hpp>
#include <kalman_filter_app/SensorFusionImuOdom.hpp>
#include <


class SensorFusionNode : public rclcpp::Node
{

public:
    SensorFusionNode(const string name) : Node(name);
    ~SensorFusionNode() = default;

    void imu_callback(const std::shared_ptr<sensor_msgs::msgs::Imu> imu_msg);
    void odom_callback(const std::shared_ptr<nav_msgs::msgs::Odometry> imu_msg);
    
private:
    rclcpp::Subscriber<sensor_msgs::msgs::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscriber<nav_msgs::msgs::Imu>::SharedPtr imu_sub_;
    sensor_fusion::SensorFusionImuOdom;

    rclcpp::Subscriber<nav_msgs::msgs::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::TimerBase::SharedPtr_ timer_;

}; //

#endif