#include <observation_model/OdomVelocityObservationModel.hpp>
#include <cmath>

observationmodel::OdomVelocityObservationModel::OdomVelocityObservationModel()
{
    observation_func = [](const Eigen::VectorXd &mu, double) -> Eigen::VectorXd
    {
        double c = std::cos(mu(2));
        double s = std::sin(mu(2));
        Eigen::VectorXd z(2);
        z(0) = c * mu(3) + s * mu(4);
        z(1) = -s * mu(3) + c * mu(4);
        return z;
    };

    observation_jac = [](const Eigen::VectorXd &mu, double) -> Eigen::MatrixXd
    {
        double c = std::cos(mu(2));
        double s = std::sin(mu(2));
        double v_fwd = c * mu(3) + s * mu(4);
        double v_lat = -s * mu(3) + c * mu(4);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, mu.size());
        H(0, 2) = v_lat;
        H(0, 3) = c;
        H(0, 4) = s;
        H(1, 2) = -v_fwd;
        H(1, 3) = -s;
        H(1, 4) = c;
        return H;
    };
}
