#pragma once
#include <Eigen/Dense>
#include "pendulum_dynamics.hpp"

struct LinearDiscreteModel
{
    Eigen::Matrix4d Ad{Eigen::Matrix4d::Zero()};
    Eigen::Vector4d Bd{Eigen::Vector4d::Zero()};
    Eigen::Matrix<double, 2, 4> C{Eigen::Matrix<double, 2, 4>::Zero()};
};