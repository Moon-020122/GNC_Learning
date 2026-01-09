#include "pendulum_ekf_model.hpp"
#include "integrators.hpp"
#include <algorithm>

PendulumEkfModel::PendulumEkfModel(PendulumParams params, double dt)
    : p_(params), dt_(dt) {}

State PendulumEkfModel::f_discrete(const State &x, double u) const
{
    return rk4_step(x, u, dt_, p_); // Solve using the rk4 integrator
}

Meas PendulumEkfModel::h_meas(const State &x) const
{
    Meas z;
    // h matrix, because it is not a left transformation or the like, but simply a mask, so it is a constant matrix
    z << x(0), x(2);
    return z;
}

Mat4 PendulumEkfModel::jacobian_F(const State &x, double u) const
{
    Mat4 F = Mat4::Zero();

    for (int i = 0; i < 4; ++i)
    {
        State xp = x;  // plus
        State xm = x;  // minus
        xp(i) += eps_; // Add or subtract an error from the state
        xm(i) -= eps_;

        const State fp = f_discrete(xp, u);
        const State fm = f_discrete(xm, u);

        F.col(i) = (fp - fm) / (2.0 * eps_);
    }
    return F;
}

Mat24 PendulumEkfModel::jacobian_H(const State &x) const
{
    // H = ∂h/∂x via Taylor expansion (finite differences)
    // NOTE: For this specific h(x) = [x, theta], H is constant:
    // [1 0 0 0; 0 0 1 0]
    // But we still compute it numerically

    Mat24 H = Mat24::Zero();

    for (int i = 0; i < 4; ++i)
    {
        State xp = x;
        State xm = x;
        xp(i) += eps_;
        xm(i) -= eps_;
        const Meas hp = h_meas(xp);
        const Meas hm = h_meas(xm);

        H.col(i) = (hp - hm) / (2.0 * eps_);
    }
    return H;
}