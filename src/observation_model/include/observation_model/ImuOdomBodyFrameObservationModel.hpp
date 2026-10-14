/**
 * @file ImuOdomBodyFrameObservationModel.hpp
 * @brief IMU + wheel odometry observation model that compares velocities and accelerations in the body frame.
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef IMUODOM_BODYFRAME_OBSERVATION_MODEL_HPP
#define IMUODOM_BODYFRAME_OBSERVATION_MODEL_HPP

#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <observation_model/baseObservationModel.hpp>

namespace observationmodel{

    // With odom pose:    z = [x, y, theta_odom, theta_imu, w_imu, v_fwd, v_lat, a_fwd, a_lat]  (9)
    // Without odom pose: z = [theta_imu, w_imu, v_fwd, v_lat, a_fwd, a_lat]                     (6)
    // v_* come from the wheel odometry twist, a_* from the IMU; both are body-frame quantities.
    class ImuOdomBodyFrameObservationModel:public BaseObservationModel{

        public:

        explicit ImuOdomBodyFrameObservationModel(bool use_odom_pose);
        ~ImuOdomBodyFrameObservationModel() = default;

        // Rows of z that hold angles (for innovation wrapping).
        std::vector<int> angle_indices;
        int dim_z;
    };

}; //observationmodel

#endif
