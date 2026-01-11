#pragma once
#include <Eigen/Dense>
#include "pendulum_params.hpp"

using State = Eigen::Matrix<double,4,1>; //[x, xdot, theta , thetadot]^T

State dynamics_f(const State &x, double u, const PendulumParams& p);