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

sensor_fusion::SensorFusionImuOdom::SensorFusionImuOdom(const Eigen::VectorXd &initial_state,
                                                        const Eigen::VectorXd &initial_cov,
                                                        const Eigen::VectorXd &proc_noise_std,
                                                        const Eigen::VectorXd &obs_noise_std,
                                                        std::shared_ptr<motionmodel::BaseMotionModel> mtn_mdl,
                                                        std::shared_ptr<observationmodel::BaseObservationModel> obs_mdl)
{
    motion_model = mtn_mdl;
    observation_model = obs_mdl;
    ekf_filter_ = std::make_shared<kalman::EKF>(initial_state, initial_cov, proc_noise_std, obs_noise_std);
    // Convert setNonlinearProcessModel
    // Direct assignment instead of std::bind
    ekf_filter_->setNonlinearProcessModel(mtn_mdl->process_model);

    // Direct assignment for Process Jacobian
    ekf_filter_->setProcessJacobian(mtn_mdl->process_jac);

    // Direct assignment for Observation Function
    ekf_filter_->setObservationFunction(obs_mdl->observation_func);

    // Direct assignment for Observation Jacobian
    ekf_filter_->setObservationJacobian(obs_mdl->observation_jac);
}