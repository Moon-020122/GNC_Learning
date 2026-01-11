#pragma once
#include <Eigen/Dense>
#include <pendulum_params.hpp>
#include <pendulum_dynamics.hpp>

using Meas = Eigen::Matrix<double, 2, 1>;
using Mat4 = Eigen::Matrix<double, 4, 4>;
using Mat42 = Eigen::Matrix<double, 4, 2>;
using Mat24 = Eigen::Matrix<double, 2, 4>;
using Mat2 = Eigen::Matrix<double, 2, 2>;

class PendulumEkfModel
{
public:
    PendulumEkfModel(PendulumParams params, double dt);

    State f_discrete(const State &x, double u) const;

    Meas h_meas(const State &x) const;
    // Jacobians (computed via Taylor expansion -> finite differences)
    Mat4 jacobian_F(const State &x, double u) const; // ∂f_d/∂x
    Mat24 jacobian_H(const State &x) const;           // ∂h/∂x

private:
    PendulumParams p_;
    double dt_{0.01};
    double eps_{1e-6}; // finite-difference step (from Taylor)
};