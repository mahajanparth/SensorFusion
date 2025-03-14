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

        kalman::EKF ekf_filter_;
        motionmodel::BaseMotionModel motion_model;
        observationmodel::BaseObservationModel observation_model;

        Sensor_Fusion() = default;
        ~Sensor_Fusion() = default;

    }; // Base_Sensor_Fusion

}; //sensor fusion 

#endif