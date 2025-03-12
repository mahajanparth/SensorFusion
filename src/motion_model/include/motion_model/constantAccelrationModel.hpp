/**
 * @file baseMotionModel.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-09
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef CONSTANT_ACCELRATION_MOTION_MODEL_HPP
#define CONSTANT_ACCELRATION_MOTION_MODEL_HPP
#include <Eigen/Dense>
#include <functional>
#include <motion_model/baseMotionModel.hpp>


namespace motionmodel{

    class ConstantAccMotionModel: public BaseMotionModel {

        public:

        ConstantAccMotionModel();
        ~ConstantAccMotionModel() = default;  
     
    };

}; //motionamodel

#endif 