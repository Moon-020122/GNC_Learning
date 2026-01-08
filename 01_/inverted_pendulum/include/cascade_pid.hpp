#pragma once
#include <cmath>
#include "pid.hpp"
#include "pendulum_dynamics.hpp" // State
#include "saturations.hpp"
#include "rate_limiter.hpp"

struct CascadePidConfig
{
    // theta_ref limits (rad)
    double theta_ref_min{-0.1745329252}; // -10 deg
    double theta_ref_max{+0.1745329252}; // +10 deg

    // theta_ref rate limit (rad/s)
    double theta_ref_rate_limit{0.5}; // e.g. 0.5 rad/s (~28.6 deg/s)

    // outer-loop decimation: update outer PID every N inner steps
    int outer_update_every{5}; // dt=0.01 => 20Hz outer loop
};

class CascadePidController
{
public:
    CascadePidController(PID pos_pid, PID ang_pid, CascadePidConfig cfg, double dt)
        : pos_pid_(std::move(pos_pid)),
          ang_pid_(std::move(ang_pid)),
          cfg_(cfg),
          dt_(dt),
          theta_rate_limiter_(cfg.theta_ref_rate_limit, 0.0) {}

    void reset()
    {
        pos_pid_.reset();
        ang_pid_.reset();
        theta_ref_ = 0.0;
        theta_rate_limiter_.reset(0.0);
        step_count_ = 0;
    }

    // Set references
    void set_x_ref(double x_ref) { x_ref_ = x_ref; }
    void set_theta_ref_limits(double min_rad, double max_rad)
    {
        cfg_.theta_ref_min = min_rad;
        cfg_.theta_ref_max = max_rad;
    }

    // Compute control u using measured state x
    // Returns u (N). Also stores theta_ref_ for logging/debug.
    double compute(const State &x)
    {
        const double x_meas = x(0);
        const double theta_meas = x(2);

        // ---- Outer loop (position) ----
        // Update slower: every N steps, otherwise hold previous theta_ref_
        if (cfg_.outer_update_every <= 1 || (step_count_ % cfg_.outer_update_every) == 0)
        {
            // pos_pid output is theta_ref command (rad), so set PID output limits to theta_ref limits
            double theta_cmd = pos_pid_.update(x_ref_, -x_meas);

            // hard clamp to +/- 10deg
            theta_cmd = clamp(theta_cmd, cfg_.theta_ref_min, cfg_.theta_ref_max);
            raw_theta_cmd_ = theta_cmd;
            // rate limit (smooth outer-loop demand)
        }
        theta_ref_ = theta_rate_limiter_.update(raw_theta_cmd_, dt_);
        // ---- Inner loop (angle) ----
        const double u = ang_pid_.update(theta_ref_, -theta_meas);

        ++step_count_;
        return u;
    }

    double theta_ref() const { return theta_ref_; }

private:
    PID pos_pid_;
    PID ang_pid_;
    CascadePidConfig cfg_;
    double dt_;

    double x_ref_{0.0};
    double theta_ref_{0.0};
    double raw_theta_cmd_;

    RateLimiter theta_rate_limiter_;
    int step_count_{0};
};
