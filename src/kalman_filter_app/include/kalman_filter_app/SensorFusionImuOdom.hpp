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

#ifndef SENSOR_FUSION_HPP
#define SENSOR_FUSION_HPP

#include <rclcpp/rclcpp>
#include <kalman_filter_core/EKFFilter.hpp>
#include <motion_model/baseMotionModel.hpp>
#include <observation_model/baseObservationModel.hpp>
#include <kalman_filter_app/BaseSensorFusion.hpp>

namespace sensor_fusion
{
    class SensorFusionImuOdom : public Base_Sensor_Fusion
    {

        SensorFusionImuOdom(const Eigen::vectorXd &initial_state,
                            const Eigen::vectorXd &initial_cov,
                            const Eigen::VectorXd &proc_noise_std,
                            const Eigen::VectorXd &obs_noise_std,
                            motionmodel::BaseMotionModel motion_model,
                            observationmodel::BaseObservationModel observation_model);
        ~SensorFusionImuOdom() = default;

    }; // Base_Sensor_Fusion

}; // sensor fusion

#endif