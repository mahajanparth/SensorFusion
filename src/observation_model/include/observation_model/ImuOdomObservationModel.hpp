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
#ifndef IMUODOMOBSERVATION_MODEL_HPP
#define IMUODOMOBSERVATION_MODEL_HPP

#include <Eigen/Dense>
#include <functional>
#include <observation_model/baseObservationModel.hpp>

namespace observationmodel{

    class ImuOdomObservationModel:public BaseObservationModel{

        public:

        ImuOdomObservationModel();
        ~ImuOdomObservationModel() = default;
        
        
    };

}; //observationmodel

#endif 