/**
 * @file EKFFilter.hpp
 * @author parth mahajan (mahajan.parth@northeastern.edu)
 * @brief 
 * @version 0.1
 * @date 
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef EKFFILTER_H
#define EKFFILTER_H

#include <Eigen/Dense>
#include <functional>


namespace kalman{

    class EKF{


        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> nonlinear_process_model;
        std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> process_jac;
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> observation_func;
        std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> observation_jac; 

        Eigen::MatrixXd q_process_noise;
        Eigen::MatrixXd r_sensor_noise;
        
        Eigen::VectorXd control_input;
        
        Eigen::VectorXd z_observation;

        Eigen::VectorXd x_prior_mean;
        Eigen::VectorXd x_posterior_mean;
        
        Eigen::MatrixXd x_prior_cov;
        Eigen::MatrixXd x_posterior_cov;

        int dim_x;
        int dim_Z;

        EKF()=default;

        public:
        
        ~EKF()=default;
        
        EKF(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov,const Eigen::VectorXd& proc_noise_std,const Eigen::VectorXd& obs_noise_std ); 
        
        std::pair<Eigen::VectorXd,Eigen::MatrixXd> predict(const Eigen::VectorXd& u, double dt=0);
        
        std::pair<Eigen::VectorXd,Eigen::MatrixXd> update(const Eigen::VectorXd& z, double dt=0);
        
        // Getter and setter for the nonlinear process model.
        void setNonlinearProcessModel(const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& model);
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& getNonlinearProcessModel() const;

        // Getter and setter for the process Jacobian.
        void setProcessJacobian(const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& jac);
        const std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)>& getProcessJacobian() const;

        // Getter and setter for the observation function.
        void setObservationFunction(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func);
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& getObservationFunction() const;

        // Getter and setter for the observation Jacobian.
        void setObservationJacobian(const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& jac);
        const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& getObservationJacobian() const;

        // Getter and setter for process noise covariance.
        void setProcessNoiseCovariance(const Eigen::MatrixXd& Q);
        const Eigen::MatrixXd& getProcessNoiseCovariance() const;

        // Getter and setter for sensor noise covariance.
        void setSensorNoiseCovariance(const Eigen::MatrixXd& R);
        const Eigen::MatrixXd& getSensorNoiseCovariance() const;

        // Getter and setter for control input.
        void setControlInput(const Eigen::VectorXd& u);
        const Eigen::VectorXd& getControlInput() const;

        // Getter and setter for observation.
        void setObservation(const Eigen::VectorXd& z);
        const Eigen::VectorXd& getObservation() const;

        // Getter and setter for the prior state mean.
        void setPriorMean(const Eigen::VectorXd& mean);
        const Eigen::VectorXd& getPriorMean() const;

        // Getter and setter for the posterior state mean.
        void setPosteriorMean(const Eigen::VectorXd& mean);
        const Eigen::VectorXd& getPosteriorMean() const;

        // Getter and setter for the prior covariance.
        void setPriorCovariance(const Eigen::MatrixXd& cov);
        const Eigen::MatrixXd& getPriorCovariance() const;

        // Getter and setter for the posterior covariance.
        void setPosteriorCovariance(const Eigen::MatrixXd& cov);
        const Eigen::MatrixXd& getPosteriorCovariance() const;

        // Getter and setter for the dimension of the state.
        void setDimX(int d);
        int getDimX() const;

        // Getter and setter for the dimension of the observation.
        void setDimZ(int d);
        int getDimZ() const;


    }

}; //kalman 


#endif 
