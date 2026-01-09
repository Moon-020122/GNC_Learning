#pragma once
#include <Eigen/Dense>
#include "pendulum_ekf_model.hpp"

class EKF
{
public:
    explicit EKF(PendulumEkfModel model);

    void reset(const State &x0, const Mat4 &P0);

    void set_Q(const Mat4 &Q);
    void set_R(const Mat2 &R);

    void predict(double u);

    void update(const Meas &z);

    const State& xhat() const{return xhat_;}
    const Mat4& P() const{return P_;}

private:
    PendulumEkfModel model_;
    State xhat_{State::Zero()};
    Mat4 P_{Mat4::Identity()}; // identity matrix
    Mat4 Q_{Mat4::Identity()};
    Mat2 R_{Mat2::Identity()};
};