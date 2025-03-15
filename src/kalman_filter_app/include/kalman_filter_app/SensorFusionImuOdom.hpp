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

#include <rclcpp/rclcpp.hpp>
#include <kalman_filter_core/EKFFilter.hpp>
#include <motion_model/baseMotionModel.hpp>
#include <observation_model/baseObservationModel.hpp>
#include <kalman_filter_app/BaseSensorFusion.hpp>
#include <motion_model/baseMotionModel.hpp>
#include <motion_model/constantAccelrationModel.hpp>

#include <observation_model/baseObservationModel.hpp>
#include <observation_model/ImuOdomObservationModel.hpp>

namespace sensor_fusion
{
    class SensorFusionImuOdom : public Base_Sensor_Fusion
    {

        public:
        SensorFusionImuOdom(const Eigen::VectorXd &initial_state,
                            const Eigen::VectorXd &initial_cov,
                            const Eigen::VectorXd &proc_noise_std,
                            const Eigen::VectorXd &obs_noise_std,
                            std::shared_ptr<motionmodel::BaseMotionModel> motion_model,
                            std::shared_ptr<observationmodel::BaseObservationModel> observation_model);
        ~SensorFusionImuOdom() = default;

    }; // Base_Sensor_Fusion

}; // sensor fusion

#endif