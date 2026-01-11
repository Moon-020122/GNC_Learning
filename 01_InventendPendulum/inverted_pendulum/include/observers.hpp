#pragma once
#include <Eigen/Dense>
#include "pendulum_dynamics.hpp"

using Meas = Eigen::Matrix<double, 2, 1>;

struct ObserverModel
{
    Eigen::Matrix4d Ad{Eigen::Matrix4d::Zero()};
    Eigen::Vector4d Bd{Eigen::Vector4d::Zero()};
    Eigen::Matrix<double, 2, 4> C{Eigen::Matrix<double, 2, 4>::Zero()};
};

// lUENBERGER OBSERVER (DISCRETE)

class LuenbergerObserver
{
public:
    ObserverModel m;
    Eigen::Matrix<double, 4, 2> L{Eigen::Matrix<double, 4, 2>::Zero()};
    State xhat{State::Zero()};

    void reset(const State &x0 = State::Zero()) { xhat = x0; }

    // xhat_{k+1} = Ad xhat_k + Bd u_k + L(y_k - C xhat_k)
    void update(double u, const Meas &y)
    {
        const State xpred = m.Ad * xhat + m.Bd * u;
        const Meas innov = y - m.C * xhat;
        xhat = xpred + L * innov;
    }
};

class SteadyKalmanFilter
{
public:
    ObserverModel m;
    Eigen::Matrix<double, 4, 2> Kk{Eigen::Matrix<double, 4, 2>::Zero()};
    State xhat{State::Zero()};

    void reset(const State &x0 = State::Zero()) { xhat = x0; }

    void update(double u, const Meas &y)
    {
        const State xpred = m.Ad * xhat + m.Bd * u;
        const Meas innov = y - m.C * xpred;
        xhat = xpred + Kk * innov;
    }
};
