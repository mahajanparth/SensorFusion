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
#include <SensorFusionNode.hpp>
#include <common_utils/rotation_helpers.hpp>


SensorFusionNode::SensorFusionNode(const string name) : Node(name)
{
    this->declare_parameter("Q_process_noise");
    this->declare_parameter("R_observation_noise");
    this->declare_parameter("X_initial_state");
    this->declare_parameter("P_initial_cov");
}

void SensorFusionNode::imu_callback(const std::shared_ptr<nav_msgs::msgs::Imu> imu_msg){

    
}

void SensorFusionNode::odom_callback(const std::shared_ptr<nav_msgs::msgs::Odometry> odom_msg) {

    if( !initial_pose_flag){
        
        initial_pose= get_normalized_pose2D(rotate_pose2D(odom_to_pose2D(imu_msg.get()),-90));
        initial_pose_flag =true;
        return;
    }
    current_pose=get_normalized_pose2D(rotate_pose2D(odom_to_pose2D(imu_msg.get()),-90));
    Vector3  linear=odom_msg->twist.twist.linear;
    Vector3  angular=odom_msg->twist.twist.angular;
    u=
    dt=0.1
    X, Cov = self.kf.predict(self.u, dt)





}
