#include "motor_model.hpp"
#include <cmath>
#include <algorithm>

namespace gnc
{

    void stepMotors(MotorState &ms, const MotorParams &mp,
                    const Eigen::Vector4d &omega_cmd, double dt)
    {
        const double tau = std::max(1e-6, mp.tau_m); // tau_m:time const parameters
        const double a = std::exp(-dt / tau);

        // Exponential exact discretization 
        ms.omega = omega_cmd + (ms.omega - omega_cmd) * a;
        //Safety : avoid tiny negative due to numeric
        for (int i = 0; i < 4; ++i)
        {
            if (!std::isfinite(ms.omega[i]) || ms.omega[i] < 0.0)
                ms.omega[i] = 0.0;
        }
    }
}