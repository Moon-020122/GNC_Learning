#pragma once
#include <Eigen/Dense>

namespace gnc
{

    struct MotorParams
    {
        // A constant that measures the response speed of a rotor
        double tau_m = 0.03;  
    };

    struct MotorState
    {
        Eigen::Vector4d omega = Eigen::Vector4d::Zero(); // rads . Quadrotor UAV
    };

    // first-oreder motor lag update (exact discretization)
    void stepMotors(MotorState &ms, const MotorParams &mp,
                    const Eigen::Vector4d &omega_cmd, double dt);
}