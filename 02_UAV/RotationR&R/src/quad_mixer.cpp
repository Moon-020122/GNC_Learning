#include "quad_mixer.hpp"
#include <cmath>
#include <algorithm>

namespace gnc
{

    QuadConfig makeQuadX(double arm, double kT, double kQ,
                         double omega_min, double omega_max)
    {
        QuadConfig cfg;
        cfg.arm = arm;  //Distance from the rotor to the center of mass
        cfg.kT = kT;
        cfg.kQ = kQ;
        cfg.omega_min = omega_min;
        cfg.omega_max = omega_max;

        // X:motors at 45 degrees (a,a)
        const double a = arm / std::sqrt(2.0);
        // x forward, y right
        // 1: front-left  ( +x, -y )
        // 2: front-right ( +x, +y )
        // 3: rear-right  ( -x, +y )
        // 4: rear-left   ( -x, -y )
        cfg.r_b = {
            Eigen::Vector3d(+a, -a, 0.0),
            Eigen::Vector3d(+a, +a, 0.0),
            Eigen::Vector3d(-a, +a, 0.0),
            Eigen::Vector3d(-a, -a, 0.0)};
        // Spin signs for yaw torque row.
        // Example convention:
        // +1 means this motor contributes +tau_z = +kQ*omega^2
        // Typical quad alternates spin directions:
        cfg.spin = {+1, -1, +1, -1};

        return cfg;
    }

    Mixer buildMixer(const QuadConfig &cfg)
    {
        Mixer mx;

        for (int i = 0; i < 4; ++i)
        {
            const double x = cfg.r_b[i].x();
            const double y = cfg.r_b[i].y();

            mx.A(0, i) = cfg.kT;
            mx.A(1, i) = -cfg.kT * y;
            mx.A(2, i) = cfg.kT * x;
            mx.A(3, i) = cfg.kQ * cfg.spin[i];
        }

        mx.A_inv = mx.A.inverse(); // Xquad should be invertible with reasonable cfg
        return mx;
    }

    Eigen::Vector4d wrenchFromOmega(const QuadConfig &cfg, const Eigen::Vector4d &omega)
    {
        Eigen::Vector4d w2;
        for (int i = 0; i < 4; ++i)
            w2[i] = omega[i] * omega[i];
        Mixer mx = buildMixer(cfg);
        return mx.A * w2; // force u
    }
    Eigen::Vector4d allocateOmegaCmd(const QuadConfig &cfg, const Mixer &mx,
                                     const Eigen::Vector4d &u_cmd,
                                     Eigen::Vector4d *u_ach_out)
    {
        Eigen::Vector4d w2 = mx.A_inv * u_cmd;

        const double w2_min = cfg.omega_min * cfg.omega_min;
        const double w2_max = cfg.omega_max * cfg.omega_max;

        // Saturate and keep non-negative to avoid NaN
        for (int i = 0; i < 4; ++i)
        {
            if (!std::isfinite(w2[i]))
                w2[i] = 0.0;
            w2[i] = std::max(0.0, w2[i]);
            w2[i] = std::min(std::max(w2[i], w2_min), w2_max);
        }
        if (u_ach_out)
        {
            *u_ach_out = mx.A * w2;
        }

        Eigen::Vector4d omega_cmd;
        for (int i = 0; i < 4; i++)
        {
            omega_cmd[i] = std::sqrt(std::max(0.0, w2[i]));
        }
        return omega_cmd;
    }
}