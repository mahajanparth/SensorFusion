#include <observation_model/ImuOdomObservationModel.hpp>

auto base_observation_model_h = [](const Eigen::VectorXd &mu, double delta_t) -> Eigen::VectorXd
{
    // Expect mu has 8 elements: [x, y, theta, v_x, v_y, w, a_x, a_y]
    // u has 2 elements: [v, w]
    double x = mu(0);
    double y = mu(1);
    double theta = mu(2);
    // v_x and v_y from state are not used in this formulation.
    double vx = mu(3);
    double vy = mu(4);
    double w = mu(5);

    double v = sqrt((vx * vx + vy * vy));
    double a_x = mu(6);
    double a_y = mu(7);

    // Control input: velocity and angular velocity.
    // double v = u(0);
    // double w = u(1);

    Eigen::VectorXd z(7);
    z(0) = x;
    z(1) = y;
    z(2) = theta;
    z(3) = theta; // theta imu
    z(4) = w;
    z(5) = a_x;
    z(6) = a_y;
    return z;
};

// Jacobian of g with respect to state.
auto jacobian_of_h_wrt_state_z = [](const Eigen::VectorXd &mu, double delta_t) -> Eigen::MatrixXd
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, 8);

    // First row
    H(0, 0) = 1.0;

    H(1, 1) = 1.0;

    H(2, 2) = 1.0;
    H(3, 2) = 1.0;

    H(4, 5) = 1.0;

    H(5, 6) = 1.0;

    H(6, 7) = 1.0;

    return H;
};

observationmodel::ImuOdomObservationModel::ImuOdomObservationModel()
{

    observation_func = base_observation_model_h;
    observation_jac = jacobian_of_h_wrt_state_z;
}