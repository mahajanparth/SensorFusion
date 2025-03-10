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

#ifndef BASE_MOTION_MODEL_HPP
#define BASE_MOTION_MODEL_HPP
#include <Eigen/Dense>
#include <functional>


namespace motionmodel{

    class BaseMotionModel{

        private:
        BaseMotionModel()  = default;
        ~BaseMotionModel() = default;

        public:

        //TODO : Convert to a template instead
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> process_model;
        std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> process_jac;
        
    }

}; //motionmodel

#endif 