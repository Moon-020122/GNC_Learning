#pragma once

#include "pendulum_dynamics.hpp"

inline State rk4_step(const State &x, double u, double dt, const PendulumParams &p)
{
    const State k1 = dynamics_f(x, u, p);
    const State k2 = dynamics_f(x + 0.5 * dt * k1, u, p);
    const State k3 = dynamics_f(x + 0.5 * dt * k2, u, p);
    const State k4 = dynamics_f(x + 1.0 * dt * k3, u, p);

    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}