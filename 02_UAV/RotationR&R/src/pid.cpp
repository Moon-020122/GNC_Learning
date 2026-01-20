#include "pid.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gnc
{
    static Eigen::VectorXd makeInfVec(int n, double val)
    {
        return Eigen::VectorXd::Constant(n, val);
    }
    Pid::Pid(int dim) : n_(dim)
    {
        if (n_ <= 0)
            throw std::invalid_argument("Pid dim must be positive");
        cfg_.kp = Eigen::VectorXd::Zero(n_);
        cfg_.ki = Eigen::VectorXd::Zero(n_);
        cfg_.kd = Eigen::VectorXd::Zero(n_);
        // Initialize all to negative infinity to ensure safety
        cfg_.u_min = makeInfVec(n_, -std::numeric_limits<double>::infinity());
        cfg_.u_max = makeInfVec(n_, -std::numeric_limits<double>::infinity());
        cfg_.i_min = makeInfVec(n_, -std::numeric_limits<double>::infinity());
        cfg_.i_max = makeInfVec(n_, -std::numeric_limits<double>::infinity());
        cfg_.kaw = Eigen::VectorXd::Zero(n_);

        reset();
    }
    void Pid::setConfig(const PidConfig &cfg)
    {
        auto checkSize = [&](const Eigen::VectorXd &v, const char *name)
        {
            if (v.size() != n_)
                throw std::invalid_argument(std::string("Pidconfig size mismatch:") + name);
        };
        checkSize(cfg.kp, "kp");
        checkSize(cfg.ki, "ki");
        checkSize(cfg.kd, "kd");
        checkSize(cfg.u_min, "u_min");
        checkSize(cfg.u_max, "u_max");
        checkSize(cfg.i_min, "i_min");
        checkSize(cfg.i_max, "i_max");
        checkSize(cfg.kaw, "kaw");

        cfg_ = cfg;
    }

    void Pid::reset()
    {
        initialized_ = false;
        e_prev_ = Eigen::VectorXd::Zero(n_);
        i_state_ = Eigen::VectorXd::Zero(n_);
        d_state_ = Eigen::VectorXd::Zero(n_);
    }

    Eigen::VectorXd Pid::clampVec(const Eigen::VectorXd &x,
                                  const Eigen::VectorXd &xmin,
                                  const Eigen::VectorXd &xmax)
    {
        Eigen::VectorXd y = x;
        for (int i = 0; i < y.size(); ++i)
        {
            const double lo = xmin[i];
            const double hi = xmax[i];
            if (y[i] < lo)
                y[i] = lo;
            if (y[i] > hi)
                y[i] = hi;
            if (!std::isfinite(y[i]))
                y[i] = 0.0;
        }
        return y;
    }

    Eigen::VectorXd Pid::step(const Eigen::VectorXd &e, double dt)
    {
        if (e.size() != n_)
            throw std::invalid_argument("Pid::step error size mismatch");
        if (!(dt > 0.0) || !std::isfinite(dt))
        {
            // bad dt: output 0 to stay safe
            return Eigen::VectorXd::Zero(n_);
        }

        // Derivative of error (raw)
        Eigen::VectorXd d_raw = Eigen::VectorXd::Zero(n_);
        if (initialized_)
        {
            d_raw = (e - e_prev_) / dt;
        }
        e_prev_ = e;
        initialized_ = true;

        // Derivative low-pass filter
        if (cfg_.enable_derivative && cfg_.d_filter_tau > 0.0)
        {
            const double alpha = std::exp(-dt / cfg_.d_filter_tau);
            d_state_ = alpha * d_state_ + (1.0 - alpha) * d_raw;
        }
        else
        {
            d_state_ = d_raw;
        }

        // PID terms (unsaturated)
        const Eigen::VectorXd u_p = cfg_.kp.cwiseProduct(e);
        const Eigen::VectorXd u_d = (cfg_.enable_derivative ? Eigen::VectorXd(cfg_.kd.cwiseProduct(d_state_))
                                                            : Eigen::VectorXd::Zero(n_));
        const Eigen::VectorXd u_i = (cfg_.enable_integrator ? Eigen::VectorXd(cfg_.ki.cwiseProduct(i_state_))
                                                            : Eigen::VectorXd::Zero(n_));

        const Eigen::VectorXd u_unsat = u_p + u_d + u_i;
        const Eigen::VectorXd u_sat = clampVec(u_unsat, cfg_.u_min, cfg_.u_max);

        // Anti-windup: back-calculation on integrator state
        if (cfg_.enable_integrator)
        {
            const Eigen::VectorXd aw = cfg_.kaw.cwiseProduct(u_sat - u_unsat);
            i_state_ += (e + aw) * dt;
            i_state_ = clampVec(i_state_, cfg_.i_max, cfg_.i_max);
        }

        return u_sat;
    }

    double Pid::step1(double e, double dt)
    {
        Eigen::VectorXd ev(1);
        ev[0] = e;
        return step(ev, dt)[0];
    }

    Eigen::Vector3d Pid::step3(const Eigen::Vector3d &e, double dt)
    {
        Eigen::VectorXd ev(3);
        ev << e.x(), e.y(), e.z();
        Eigen::VectorXd uv = step(ev, dt);
        return Eigen::Vector3d(uv[0],uv[1],uv[2]);
    }
}
