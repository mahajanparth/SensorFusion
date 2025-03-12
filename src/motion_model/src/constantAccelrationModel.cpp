/**
 * @file constantAccelrationModelModel.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-09
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <motion_model/constantAccelrationModel.hpp>

auto state_transition_function_g = [](const Eigen::VectorXd &mu, const Eigen::VectorXd &u, double delta_t) -> Eigen::VectorXd
{
    // Expect mu has 8 elements: [x, y, theta, v_x, v_y, w, a_x, a_y]
    // u has 2 elements: [v, w]
    double x = mu(0);
    double y = mu(1);
    double theta = mu(2);
    // v_x and v_y from state are not used in this formulation.
    double vx = mu(3);
    double vy = mu(4);

    double v = sqrt((vx * vx + vy * vy));
    double w = mu(5);
    double a_x = mu(6);
    double a_y = mu(7);

    // Control input: velocity and angular velocity.
    // double v = u(0);
    // double w = u(1);

    Eigen::VectorXd g(8);
    g(0) = x + v * std::cos(theta) * delta_t + 0.5 * a_x * delta_t * delta_t;
    g(1) = y + v * std::sin(theta) * delta_t + 0.5 * a_y * delta_t * delta_t;
    g(2) = theta + w * delta_t;
    g(3) = v * std::cos(theta) + a_x * delta_t;
    g(4) = v * std::sin(theta) + a_y * delta_t;
    g(5) = w;
    g(6) = a_x;
    g(7) = a_y;
    return g;
};

// Jacobian of g with respect to state.
auto jacobian_of_g_wrt_state_G = [](const Eigen::VectorXd &mu, const Eigen::VectorXd &u, double delta_t) -> Eigen::MatrixXd
{
    double theta = mu(2);

    double vx = mu(3);
    double vy = mu(4);

    double v = sqrt((vx * vx + vy * vy));
    double w = mu(5);
    // double w = u(1); // not used in Jacobian computations below
    double a_x = mu(6);
    double a_y = mu(7);
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(8, 8);

    // First row
    G(0, 0) = 1.0;
    G(0, 2) = -delta_t * v * std::sin(theta);
    G(0, 6) = 0.5 * delta_t * delta_t;

    // Second row
    G(1, 1) = 1.0;
    G(1, 2) = delta_t * v * std::cos(theta);
    G(1, 7) = 0.5 * delta_t * delta_t;

    // Third row
    G(2, 2) = 1.0;
    G(2, 5) = delta_t;

    // Fourth row
    G(3, 2) = -v * std::sin(theta);
    G(3, 6) = delta_t;

    // Fifth row
    G(4, 2) = v * std::cos(theta);
    G(4, 7) = delta_t;

    // Sixth row
    G(5, 5) = 1.0;

    // Seventh row
    G(6, 6) = 1.0;

    // Eighth row
    G(7, 7) = 1.0;

    return G;
};

motionmodel::ConstantAccMotionModel::ConstantAccMotionModel()
{
    process_model = state_transition_function_g;
    process_jac = jacobian_of_g_wrt_state_G;
}