/**
 * @file baseObservationModel.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BASE_OBSERVATION_MODEL_HPP
#define BASE_OBSERVATION_MODEL_HPP
#include <Eigen/Dense>
#include <functional>


namespace observationmodel{

    class BaseObservationModel{

        
        public:

        BaseObservationModel()  = default;
        ~BaseObservationModel() = default;
        
        //TODO : Convert to a template instead
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> observation_func;
        std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> observation_jac;
        
    }

}; //motionmodel

#endif 