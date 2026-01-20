#pragma once
#include <Eigen/Dense>

namespace gnc
{

    struct PidConfig
    {
        Eigen::VectorXd kp;
        Eigen::VectorXd ki;
        Eigen::VectorXd kd;

        // Outpus saturation (per- axis) Use +/-inf for no limit
        Eigen::VectorXd u_min;
        Eigen::VectorXd u_max;

        // Integrator state saturation (per-axis) applies to integral of error(not mutiplied by ki)
        Eigen::VectorXd i_min;
        Eigen::VectorXd i_max;

        // Anti-windup back-calculation gain. 0 disables back-calc. Typical small positive
        Eigen::VectorXd kaw; // Artificially set parameters

        // Derivative low-pass filter time constant. 0 disables filter
        double d_filter_tau = 0.02;

        bool enable_integrator = true;
        bool enable_derivative = true;
    };

    class Pid
    {
    public:
        explicit Pid(int dim = 1);

        void setConfig(const PidConfig &cfg);
        const PidConfig &config() const { return cfg_; }

        void reset();

        // core step: input error vector e(size N), dt(s), returns saturated output u (size N)
        Eigen::VectorXd step(const Eigen::VectorXd &e, double dt);

        // convenience for 1d and 3d
        double step1(double e, double dt);
        Eigen::Vector3d step3(const Eigen::Vector3d &e, double dt);

        // Access internal states for debugging
        Eigen::VectorXd integratorState() const { return i_state_; }
        Eigen::VectorXd derivativeState() const { return d_state_; }

    private:
        int n_ = 1;
        PidConfig cfg_;

        bool initialized_ = false;
        Eigen::VectorXd e_prev_;
        Eigen::VectorXd i_state_; // integral of error;
        Eigen::VectorXd d_state_; // filtered derivative of error

        static Eigen::VectorXd clampVec(
            const Eigen::VectorXd &x,
            const Eigen::VectorXd &xmin,
            const Eigen::VectorXd &xmax);
    };
}