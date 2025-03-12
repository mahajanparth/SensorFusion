/**
 * @file EKFFilter.cpp
 * @author parth mahajan (mahajan.parth@northeastern.edu)
 * @brief
 * @version 0.1
 * @date
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <kalman_filter_core/EKFFilter.hpp>

kalman::EKF::EKF(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_cov, const Eigen::VectorXd &proc_noise_std, const Eigen::VectorXd &obs_noise_std) : x_prior_mean{initial_state}, x_prior_cov{initial_cov}, q_process_noise{proc_noise_std}, r_sensor_noise{obs_noise_std}
{
    nonlinear_process_model = nullptr;
    process_jac = nullptr;
    observation_func = nullptr;
    observation_jac = nullptr;
    dim_x = q_process_noise.rows();
    dim_z = r_sensor_noise.rows();

    x_posterior_mean = x_prior_mean;
    x_posterior_cov = x_prior_cov;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> kalman::EKF::predict(const Eigen::VectorXd &u, double dt)
{
    x_prior_mean = nonlinear_process_model(x_prior_mean, u, dt);
    Eigen::MatrixXd G_jac = process_jac(x_prior_mean, u, dt);
    x_prior_cov = G_jac * x_prior_cov * G_jac.transpose() + q_process_noise;
    return { x_prior_mean, x_prior_cov };
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> kalman::EKF::update(const Eigen::VectorXd &z, double dt)
{
    Eigen::MatrixXd H_jac = observation_jac(x_prior_mean);

    Eigen::MatrixXd S = H_jac * x_prior_cov * H_jac.transpose() + r_sensor_noise;

    Eigen::MatrixXd K = x_prior_cov * H_jac.transpose() * S.inverse();

    x_posterior_mean = x_prior_mean + K * (z - observation_func(x_prior_mean));

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_prior_cov.rows(), x_prior_cov.cols());
    x_posterior_cov = (I - K * H_jac) * x_prior_cov;

    x_prior_mean = x_posterior_mean;
    x_prior_cov = x_posterior_cov;

    return { x_posterior_mean, x_posterior_cov };
}

// Getter and setter for the nonlinear process model.
void kalman::EKF::setNonlinearProcessModel(const std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &, double)> &model)
{
    nonlinear_process_model = model;
}
const std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &, double)> &kalman::EKF::getNonlinearProcessModel() const
{
    return nonlinear_process_model;
}

// Getter and setter for the process Jacobian.
void kalman::EKF::setProcessJacobian(const std::function<Eigen::MatrixXd(const Eigen::VectorXd &, const Eigen::VectorXd &, double)> &jac)
{
    process_jac = jac;
}
const std::function<Eigen::MatrixXd(const Eigen::VectorXd &, const Eigen::VectorXd &, double)> &kalman::EKF::getProcessJacobian() const
{
    return process_jac;
}

// Getter and setter for the observation function.
void kalman::EKF::setObservationFunction(const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &func)
{
    observation_func = func;
}
const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &kalman::EKF::getObservationFunction() const
{
    return observation_func;
}

// Getter and setter for the observation Jacobian.
void kalman::EKF::setObservationJacobian(const std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> &jac)
{
    observation_jac = jac;
}
const std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> &kalman::EKF::getObservationJacobian() const
{
    return observation_jac;
}

// Getter and setter for process noise covariance.
void kalman::EKF::setProcessNoiseCovariance(const Eigen::MatrixXd &Q)
{
    q_process_noise = Q;
}
const Eigen::MatrixXd &kalman::EKF::getProcessNoiseCovariance() const
{
    return q_process_noise;
}

// Getter and setter for sensor noise covariance.
void kalman::EKF::setSensorNoiseCovariance(const Eigen::MatrixXd &R)
{
    r_sensor_noise = R;
}
const Eigen::MatrixXd &kalman::EKF::getSensorNoiseCovariance() const
{
    return r_sensor_noise;
}

// Getter and setter for control input.
void kalman::EKF::setControlInput(const Eigen::VectorXd &u)
{
    control_input = u;
}
const Eigen::VectorXd &kalman::EKF::getControlInput() const
{
    return control_input;
}

// Getter and setter for observation.
void kalman::EKF::setObservation(const Eigen::VectorXd &z)
{
    z_observation = z;
}
const Eigen::VectorXd &kalman::EKF::getObservation() const
{
    return z_observation;
}

// Getter and setter for the prior state mean.
void kalman::EKF::setPriorMean(const Eigen::VectorXd &mean)
{
    x_prior_mean = mean;
}
const Eigen::VectorXd &kalman::EKF::getPriorMean() const
{
    return x_prior_mean;
}

// Getter and setter for the posterior state mean.
void kalman::EKF::setPosteriorMean(const Eigen::VectorXd &mean)
{
    x_posterior_mean = mean;
}
const Eigen::VectorXd &kalman::EKF::getPosteriorMean() const
{
    return x_posterior_mean;
}

// Getter and setter for the prior covariance.
void kalman::EKF::setPriorCovariance(const Eigen::MatrixXd &cov)
{
    x_prior_cov = cov;
}
const Eigen::MatrixXd &kalman::EKF::getPriorCovariance() const
{
    return x_prior_cov;
}

// Getter and setter for the posterior covariance.
void kalman::EKF::setPosteriorCovariance(const Eigen::MatrixXd &cov)
{
    x_posterior_cov = cov;
}
const Eigen::MatrixXd &kalman::EKF::getPosteriorCovariance() const
{
    return x_posterior_cov;
}

// Getter and setter for the dimension of the state.
void kalman::EKF::setDimX(int d)
{
    dim_x = d;
}
int kalman::EKF::getDimX() const
{
    return dim_x;
}

// Getter and setter for the dimension of the observation.
void kalman::EKF::setDimZ(int d)
{
    dim_z = d;
}
int kalman::EKF::getDimZ() const
{
    return dim_z;
}
