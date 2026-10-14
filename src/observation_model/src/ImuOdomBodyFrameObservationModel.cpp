#include <observation_model/ImuOdomBodyFrameObservationModel.hpp>
#include <cmath>

namespace
{
    // Body-frame part shared by both variants: [theta_imu, w, v_fwd, v_lat, a_fwd, a_lat].
    Eigen::VectorXd body_frame_h(const Eigen::VectorXd &mu)
    {
        double theta = mu(2);
        double c = std::cos(theta);
        double s = std::sin(theta);

        Eigen::VectorXd z(6);
        z(0) = theta;
        z(1) = mu(5);
        z(2) = c * mu(3) + s * mu(4);
        z(3) = -s * mu(3) + c * mu(4);
        z(4) = c * mu(6) + s * mu(7);
        z(5) = -s * mu(6) + c * mu(7);
        return z;
    }

    Eigen::MatrixXd body_frame_H(const Eigen::VectorXd &mu)
    {
        double theta = mu(2);
        double c = std::cos(theta);
        double s = std::sin(theta);
        double v_fwd = c * mu(3) + s * mu(4);
        double v_lat = -s * mu(3) + c * mu(4);
        double a_fwd = c * mu(6) + s * mu(7);
        double a_lat = -s * mu(6) + c * mu(7);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 8);
        H(0, 2) = 1.0;
        H(1, 5) = 1.0;

        H(2, 2) = v_lat;
        H(2, 3) = c;
        H(2, 4) = s;

        H(3, 2) = -v_fwd;
        H(3, 3) = -s;
        H(3, 4) = c;

        H(4, 2) = a_lat;
        H(4, 6) = c;
        H(4, 7) = s;

        H(5, 2) = -a_fwd;
        H(5, 6) = -s;
        H(5, 7) = c;
        return H;
    }
}

observationmodel::ImuOdomBodyFrameObservationModel::ImuOdomBodyFrameObservationModel(bool use_odom_pose)
{
    if (use_odom_pose)
    {
        dim_z = 9;
        angle_indices = {2, 3};
        observation_func = [](const Eigen::VectorXd &mu, double) -> Eigen::VectorXd
        {
            Eigen::VectorXd z(9);
            z << mu(0), mu(1), mu(2), body_frame_h(mu);
            return z;
        };
        observation_jac = [](const Eigen::VectorXd &mu, double) -> Eigen::MatrixXd
        {
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 8);
            H(0, 0) = 1.0;
            H(1, 1) = 1.0;
            H(2, 2) = 1.0;
            H.bottomRows(6) = body_frame_H(mu);
            return H;
        };
    }
    else
    {
        dim_z = 6;
        angle_indices = {0};
        observation_func = [](const Eigen::VectorXd &mu, double) -> Eigen::VectorXd
        { return body_frame_h(mu); };
        observation_jac = [](const Eigen::VectorXd &mu, double) -> Eigen::MatrixXd
        { return body_frame_H(mu); };
    }
}
