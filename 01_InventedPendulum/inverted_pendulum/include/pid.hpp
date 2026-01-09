#pragma once
#include <cmath>
#include "saturations.hpp"

struct PidGains
{
    double Kp{0.0};
    double Ki{0.0};
    double Kd{0.0};
};

class PID
{
public:
    PID(PidGains g, double dt, double u_min, double u_max)
        : g_(g), dt_(dt), u_min_(u_min), u_max_(u_max) {};
    // derivative-on-measurement + 1st order LPF
    void set_derivative_filter_tau(double tau) { tau_ = tau; }
    void reset()
    {
        integ_ = 0.0;
        prev_meas_ = 0.0;
        d_filt_ = 0.0;
        first_ = true;
    }
    //此处逻辑是负反馈，但是倒立摆小车比较特殊是正反馈，即往右倒车往右跑
    double update(double setpoint, double meas)
    {
        const double e = setpoint - meas;

        // derivative on measturement

        double d = 0.0;
        if (first_)
        {
            d = 0.0;
            first_ = false;
        }
        else
        {
            d = -(meas - prev_meas_) / dt_;
        }
        // low_pass filter on derivative
        const double alpha = (tau_ <= 0.0) ? 0.0 : (tau_ / (tau_ + dt_));
        d_filt_ = alpha * d_filt_ + (1.0 - alpha) * d;

        const double p_term = g_.Kp * e;
        const double d_term = g_.Kd * d_filt_;
        double u_no_i = p_term + d_term + g_.Ki * integ_;

        // saturation
        double u = clamp(u_no_i, u_min_, u_max_);

        // conditional integration (anti-windup)

        const bool sat_hi = (u >= u_max_ - 1e-12);
        const bool sat_lo = (u <= u_min_ + 1e-12);
        const bool pushing_hi = (e > 0.0);
        const bool pushing_lo = (e < 0.0);

        if (!((sat_hi && pushing_hi) || (sat_lo && pushing_lo)))
        {
            integ_ += e * dt_;
        }
        u = clamp(p_term + d_term + g_.Ki * integ_, u_min_, u_max_);

        prev_meas_ = meas;
        return u;
    }

private:
    PidGains g_;
    double dt_{0.01};
    double u_min_{-10.0}, u_max_{10.0};
    double tau_{0.02}; // derivative filter time constant
    double integ_{0.0};
    double prev_meas_{0.0};
    double d_filt_{0.0};
    bool first_{true};
};