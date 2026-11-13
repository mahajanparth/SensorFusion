/**
 * @file imuDrivenMotionModel.cpp
 * @brief
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <motion_model/imuDrivenMotionModel.hpp>
#include <cmath>

namespace
{
    // Bias-corrected IMU input rotated into the world frame.
    struct WorldAccel
    {
        double c, s, ax, ay;
    };

    WorldAccel world_accel(const Eigen::VectorXd &mu, const Eigen::VectorXd &u)
    {
        double c = std::cos(mu(2));
        double s = std::sin(mu(2));
        double a_f = u(1) - mu(6);
        double a_l = u(2) - mu(7);
        return {c, s, c * a_f - s * a_l, s * a_f + c * a_l};
    }
}

motionmodel::ImuDrivenMotionModel::ImuDrivenMotionModel(double gyro_noise_density, double accel_noise_density,
                                                        const Eigen::MatrixXd &q_continuous)
{
    process_model = [](const Eigen::VectorXd &mu, const Eigen::VectorXd &u, double dt) -> Eigen::VectorXd
    {
        auto a = world_accel(mu, u);
        Eigen::VectorXd g = mu;
        g(0) = mu(0) + mu(3) * dt + 0.5 * a.ax * dt * dt;
        g(1) = mu(1) + mu(4) * dt + 0.5 * a.ay * dt * dt;
        g(2) = mu(2) + (u(0) - mu(5)) * dt;
        g(3) = mu(3) + a.ax * dt;
        g(4) = mu(4) + a.ay * dt;
        return g;
    };

    process_jac = [](const Eigen::VectorXd &mu, const Eigen::VectorXd &u, double dt) -> Eigen::MatrixXd
    {
        auto a = world_accel(mu, u);
        double h = 0.5 * dt * dt;
        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(8, 8);
        // d a_world / d theta = [-a_y, a_x]; d a_world / d [b_af, b_al] = -R(theta).
        G(0, 2) = -a.ay * h;
        G(0, 3) = dt;
        G(0, 6) = -a.c * h;
        G(0, 7) = a.s * h;

        G(1, 2) = a.ax * h;
        G(1, 4) = dt;
        G(1, 6) = -a.s * h;
        G(1, 7) = -a.c * h;

        G(2, 5) = -dt;

        G(3, 2) = -a.ay * dt;
        G(3, 6) = -a.c * dt;
        G(3, 7) = a.s * dt;

        G(4, 2) = a.ax * dt;
        G(4, 6) = -a.s * dt;
        G(4, 7) = -a.c * dt;
        return G;
    };

    Eigen::Vector3d input_psd(gyro_noise_density * gyro_noise_density,
                              accel_noise_density * accel_noise_density,
                              accel_noise_density * accel_noise_density);
    process_noise = [input_psd, q_continuous](const Eigen::VectorXd &mu, const Eigen::VectorXd &, double dt) -> Eigen::MatrixXd
    {
        Eigen::MatrixXd Q = q_continuous * dt;
        if (dt <= 0.0)
            return Q;
        double c = std::cos(mu(2));
        double s = std::sin(mu(2));
        double h = 0.5 * dt * dt;
        // Jacobian of the motion model w.r.t. the input [w, a_fwd, a_lat].
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(8, 3);
        V(2, 0) = dt;
        V(0, 1) = c * h;
        V(1, 1) = s * h;
        V(3, 1) = c * dt;
        V(4, 1) = s * dt;
        V(0, 2) = -s * h;
        V(1, 2) = c * h;
        V(3, 2) = -s * dt;
        V(4, 2) = c * dt;
        // A white-noise density n gives a per-sample variance n^2 / dt over a step of length dt.
        Q += V * (input_psd / dt).asDiagonal() * V.transpose();
        return Q;
    };
}
