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
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionNode>("IMUODOMNODE"));
    rclcpp::shutdown();
    return 0;
}