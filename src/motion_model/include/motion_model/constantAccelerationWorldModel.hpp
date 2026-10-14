/**
 * @file constantAccelerationWorldModel.hpp
 * @brief Constant acceleration model with velocity and acceleration expressed in the world frame.
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef CONSTANT_ACCELERATION_WORLD_MOTION_MODEL_HPP
#define CONSTANT_ACCELERATION_WORLD_MOTION_MODEL_HPP

#include <Eigen/Dense>
#include <functional>
#include <motion_model/baseMotionModel.hpp>

namespace motionmodel{

    // State: [x, y, theta, v_x, v_y, w, a_x, a_y], all velocities/accelerations in the world frame.
    // Unlike ConstantAccMotionModel, v_x / v_y keep their sign (reversing is representable).
    class ConstantAccWorldMotionModel: public BaseMotionModel {

        public:

        ConstantAccWorldMotionModel();
        ~ConstantAccWorldMotionModel() = default;

    };

}; //motionmodel

#endif
