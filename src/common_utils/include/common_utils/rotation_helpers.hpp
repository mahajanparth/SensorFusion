

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cmath>
#include <tuple>

std::tuple<double, double, double> odom_to_pose2D(const nav_msgs::msg::Odometry::SharedPtr &odom)
{
    std::tuple<double, double, double> x_y_yaw = {0, 0, 0};
    std::get<0>(x_y_yaw) = odom->pose.pose.position.x;
    std::get<1>(x_y_yaw) = odom->pose.pose.position.y;
    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::get<2>(x_y_yaw) = yaw;

    return x_y_yaw;
}


double quat_to_yaw(const geometry_msgs::msg::Quaternion quat)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}



double normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

std::tuple<double, double, double> get_normalize_pose2D(const std::tuple<double, double, double> &initial_pose, const std::tuple<double, double, double> &current_pose)
{

    auto [x, y, yaw] = current_pose;
    auto [init_x, init_y, init_yaw] = initial_pose;
    x -= init_x;
    y -= init_y;
    yaw -= init_yaw;

    yaw = normalize_angle(yaw);
    return {x, y, yaw};
}


// Function to convert degrees to radians
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Function to convert radians to degrees
double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

std::tuple<double, double, double> rotate_pose2D(const std::tuple<double, double, double>& pose, double degrees) {
    // Convert degrees to radians
    double radians = degreesToRadians(degrees);

    // Extract (x, y, theta) from the tuple
    double x = std::get<0>(pose);
    double y = std::get<1>(pose);
    double theta = std::get<2>(pose);

    // Rotation matrix computation
    double cos_theta = std::cos(radians);
    double sin_theta = std::sin(radians);

    // Apply the rotation matrix to the position
    double rotated_x = cos_theta * x - sin_theta * y;
    double rotated_y = sin_theta * x + cos_theta * y;

    // Rotate the orientation, ensuring it wraps correctly
    double rotated_orientation = std::fmod(theta + radians, 2 * M_PI);

    return std::make_tuple(rotated_x, rotated_y, rotated_orientation);
}