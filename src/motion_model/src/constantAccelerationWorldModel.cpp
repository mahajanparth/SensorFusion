/**
 * @file constantAccelerationWorldModel.cpp
 * @brief
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <motion_model/constantAccelerationWorldModel.hpp>

auto world_state_transition_function_g = [](const Eigen::VectorXd &mu, const Eigen::VectorXd & /*u*/, double delta_t) -> Eigen::VectorXd
{
    // mu: [x, y, theta, v_x, v_y, w, a_x, a_y]
    Eigen::VectorXd g = mu;
    g(0) = mu(0) + mu(3) * delta_t + 0.5 * mu(6) * delta_t * delta_t;
    g(1) = mu(1) + mu(4) * delta_t + 0.5 * mu(7) * delta_t * delta_t;
    g(2) = mu(2) + mu(5) * delta_t;
    g(3) = mu(3) + mu(6) * delta_t;
    g(4) = mu(4) + mu(7) * delta_t;
    return g;
};

// The world-frame model is linear, so the Jacobian does not depend on mu.
auto world_jacobian_of_g_wrt_state_G = [](const Eigen::VectorXd & /*mu*/, const Eigen::VectorXd & /*u*/, double delta_t) -> Eigen::MatrixXd
{
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(8, 8);
    G(0, 3) = delta_t;
    G(0, 6) = 0.5 * delta_t * delta_t;
    G(1, 4) = delta_t;
    G(1, 7) = 0.5 * delta_t * delta_t;
    G(2, 5) = delta_t;
    G(3, 6) = delta_t;
    G(4, 7) = delta_t;
    return G;
};

motionmodel::ConstantAccWorldMotionModel::ConstantAccWorldMotionModel()
{
    process_model = world_state_transition_function_g;
    process_jac = world_jacobian_of_g_wrt_state_G;
}
