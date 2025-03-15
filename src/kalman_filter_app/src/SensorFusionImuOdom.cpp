/**
 * @file SensorFusion.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-03-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <kalman_filter_app/SensorFusionImuOdom.hpp>

SensorFusionImuOdom::SensorFusionImuOdom(const Eigen::vectorXd &initial_state,
                                         const Eigen::vectorXd &initial_cov,
                                         const Eigen::VectorXd &proc_noise_std,
                                         const Eigen::VectorXd &obs_noise_std,
                                         std::shared_ptr<motionmodel::BaseMotionModel> mtn_mdl,
                                         std::shared_ptr<observationmodel::BaseObservationModel> obs_mdl)
{
    motion_model = mtn_mdl;
    observation_model = obs_mdl;
    ekf_filter_ = std::make_shared<kalman::EKF>(initial_state, initial_cov, proc_noise_std, obs_noise_std, mtn_mdl, obs_mdl);
    ekf_filter_->setNonlinearProcessModel(std::bind(motionmodel::BaseMotionModel::process_model, *mtn_mdl, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    ekf_filter_->setProcessJacobian(std::bind(motionmodel::BaseMotionModel::process_jac, *mtn_mdl, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    ekf_filter_->setObservationFunction(std::bind(motionmodel::BaseObservationModel::observation_func, *obs_mdl, std::placeholders::_1, std::placeholders::_2));
    ekf_filter_->setObservationJacobian(std::bind(motionmodel::BaseObservationModel::observation_jac, *obs_mdl, std::placeholders::_1, std::placeholders::_2));
}