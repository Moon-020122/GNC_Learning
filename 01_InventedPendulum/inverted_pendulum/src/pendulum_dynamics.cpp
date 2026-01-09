#include "pendulum_dynamics.hpp"
#include <cmath>

State dynamics_f(const State &x, double u, const PendulumParams &p)
{
    const double X = x(0);
    const double Xdot = x(1);
    const double th = x(2);
    const double thdot = x(3);

    (void)X;

    const double s = std::sin(th);
    const double c = std::cos(th);

    const double u_eff = u - p.b_x * Xdot;

    // thdd = ((M+m))g sin(th) - cos (th) * (u+m l thodt^2 sin(th)) / (l*(M+m sin^2(th)))

    const double denom_th = p.l * (p.M + p.m * s * s);
    const double num_th =
        (p.M + p.m) * p.g * s - c * (u_eff + p.m * p.l * thdot * thdot * s);
    const double thdd_nom = num_th / denom_th; // 角加速度
    const double thdd = thdd_nom - p.c_th * thdot;
    // xdd = (u + m l thdot^2 sin(th) - m l thdd cos(th)) / (M+m)
    const double xdd = (u_eff + p.m * p.l * thdot * thdot * s - p.m * p.l * thdd * c) / (p.M + p.m); // 加速度

    State xdot;
    xdot << Xdot,xdd,thdot,thdd;  //状态方程

    return xdot;
};