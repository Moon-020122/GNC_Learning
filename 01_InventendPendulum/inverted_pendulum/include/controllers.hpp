#pragma once
#include <Eigen/Dense>
#include "saturations.hpp"
#include "pendulum_dynamics.hpp"
#include "pendulum_params.hpp"
#include "pid.hpp"

struct LQRController
{
    //    K << -100.0000, -51.4701, -219.4251, -39.8832;
    Eigen::RowVector4d K{Eigen::RowVector4d::Zero()};
    State x_ref{State::Zero()};
    double u_min{-10.0};
    double u_max{+10.0};

    double compute(const State &x) const
    {
        const State e = x - x_ref;
        const double u = -(K * e)(0);
        return clamp(u, u_min, u_max);
    }
};

struct AnglePidController
{
    PID pid;
    double theta_ref{0.0};

    double compute(const State &x)
    {
        const double theta = x(2);
        // Wrap angle error to [-pi, pi] so the controller targets the upright branch.
        //此处相当于把正反馈变为负反馈了，然后再在update里面变为正反馈
        double theta_mod = std::atan2(std::sin(theta_ref - theta), std::cos(theta_ref - theta));
        return pid.update(theta_ref, theta_mod);
    }
};
