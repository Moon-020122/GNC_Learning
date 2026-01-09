#include "ekf.hpp"
#include <Eigen/Dense>

EKF::EKF(PendulumEkfModel model) : model_(std::move(model)) {}

void EKF::reset(const State &x0, const Mat4 &P0)
{
    xhat_ = x0;
    P_ = P0;
}

void EKF::set_Q(const Mat4 &Q) { Q_ = Q; }
void EKF::set_R(const Mat2 &R) { R_ = R; }

void EKF::predict(double u)
{
    // /hatx^-
    const State xhat_pred = model_.f_discrete(xhat_, u);
    // xhat_ is the posterior predictive value from the previous moment
    const Mat4 F = model_.jacobian_F(xhat_, u);
    // /P^-
    Mat4 P_pred = F * P_ * F.transpose() + Q_;
    // keep symmetry (numerical hygiene)
    P_pred = 0.5 * (P_pred + P_pred.transpose());

    xhat_ = xhat_pred;
    P_ = P_pred;
}
// An update function requires observed values (i.e. sensor readings) to correct errors.
void EKF::update(const Meas &z)
{
    // obtain the predicted observed values
    const Meas zhat = model_.h_meas(xhat_);
    // INNOVATION
    const Meas y = z - zhat;
    // calculate the H matrix based on xhat_
    const Mat24 H = model_.jacobian_H(xhat_);
    // innovation covaiance S
    const Mat2 S = H * P_ * H.transpose() + R_;
    // kalman gain
    const Mat42 K = P_ * H.transpose() * S.inverse();
    // state update
    xhat_ = xhat_ + K * y;
    // converiance update (joseph form for numerical stability)
    const Mat4 I = Mat4::Identity();
    const Mat4 KH = K * H;
    Mat4 P_new = (I - KH) * P_ * (I - KH).transpose() + K*R_*K.transpose();
    P_new = 0.5* (P_new + P_new.transpose());

    P_ = P_new;
}