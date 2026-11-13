/**
 * @file imuDrivenMotionModel.hpp
 * @brief Strapdown-style planar motion model driven by the IMU (gyro + accelerometer as control input).
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef IMU_DRIVEN_MOTION_MODEL_HPP
#define IMU_DRIVEN_MOTION_MODEL_HPP

#include <Eigen/Dense>
#include <functional>
#include <motion_model/baseMotionModel.hpp>

namespace motionmodel{

    // State: [x, y, theta, v_x, v_y, b_w, b_af, b_al]
    //   position and velocity in the world frame, b_w gyro bias, b_af / b_al body-frame accelerometer biases.
    // Input: u = [w, a_fwd, a_lat] from the IMU (gravity already removed).
    //
    // The IMU is integrated instead of being used as a measurement:
    //   theta' = theta + (w - b_w) dt
    //   a_world = R(theta) [a_fwd - b_af, a_lat - b_al]
    //   p' = p + v dt + 0.5 a_world dt^2,   v' = v + a_world dt
    //
    // Process noise: the IMU noise densities [n_w, n_a] are mapped through the input Jacobian V,
    // Q = V diag(n_w^2, n_a^2, n_a^2) V^T / dt, plus q_continuous * dt for the remaining states
    // (bias random walk, model error). Both terms scale with dt, so the result does not depend on the IMU rate.
    class ImuDrivenMotionModel: public BaseMotionModel {

        public:

        ImuDrivenMotionModel(double gyro_noise_density, double accel_noise_density,
                             const Eigen::MatrixXd &q_continuous);
        ~ImuDrivenMotionModel() = default;

    };

}; //motionmodel

#endif
