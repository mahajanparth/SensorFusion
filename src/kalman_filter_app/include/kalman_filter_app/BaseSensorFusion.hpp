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

#ifndef BASESENSOR_FUSION_HPP
#define BASESENSOR_FUSION_HPP

#include <rclcpp/rclcpp>
#include <kalman_filter_core/EKFFilter.hpp>
#include <motion_model/baseMotionModel.hpp>
#include <observation_model/baseObservationModel.hpp>

namespace sensor_fusion
{
    class Base_Sensor_Fusion
    {

        public:
        std::shared_ptr<kalman::EKF> ekf_filter_;
        std::shared_ptr<motionmodel::BaseMotionModel> motion_model;
        std::shared_ptr<observationmodel::BaseObservationModel> observation_model;

        Sensor_Fusion() = default;
        ~Sensor_Fusion() = default;

    }; // Base_Sensor_Fusion

}; //sensor fusion 

#endif