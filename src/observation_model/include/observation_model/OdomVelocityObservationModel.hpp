/**
 * @file OdomVelocityObservationModel.hpp
 * @brief Wheel odometry body-frame velocity as the only measurement.
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef ODOM_VELOCITY_OBSERVATION_MODEL_HPP
#define ODOM_VELOCITY_OBSERVATION_MODEL_HPP

#include <Eigen/Dense>
#include <functional>
#include <observation_model/baseObservationModel.hpp>

namespace observationmodel{

    // z = [v_fwd, v_lat] from the wheel odometry twist, compared with the world-frame state velocity
    // (indices 3, 4) rotated into the body frame by theta (index 2). Works with any state that uses
    // that layout, e.g. motionmodel::ImuDrivenMotionModel.
    class OdomVelocityObservationModel:public BaseObservationModel{

        public:

        OdomVelocityObservationModel();
        ~OdomVelocityObservationModel() = default;

        static constexpr int dim_z = 2;
    };

}; //observationmodel

#endif
