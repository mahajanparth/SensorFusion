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

#include <rclcpp/rclcpp.hpp>
#include <kalman_filter_app/SensorFusionImuOdom.hpp>


class FuseDataNode : public rclcpp::Node
{

public:
    FuseDataNode(const string name) : Node(name)
    {
    }
    ~FuseDataNode() = default;

private:
    rclcpp::Subscriber<nav_msgs::msgs::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscriber<nav_msgs::msgs::Imu>::SharedPtr imu_sub_;
    sensor_fusion::SensorFusionImuOdom;

    rclcpp::Subscriber<nav_msgs::msgs::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::TimerBase::SharedPtr_ timer_;

}