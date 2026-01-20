#pragma once
#include <Eigen/Dense>

namespace gnc
{
    struct RigidBodyParams
    {

        Eigen::Matrix3d I_b = Eigen::Matrix3d::Identity();     // intertia in body
        Eigen::Matrix3d I_b_inv = Eigen::Matrix3d::Identity(); // precomputed inverse
        // Rotor gyro simplified model
        double rotor_J = 0.0;        // rotor rotational inertia (kg*m^2)
        // How much "net angular momentum" remains unoffset among all the rotating rotors on the entire aircraft?
        // gyroscopic moment
        double omega_spin_sum = 0.0; // sum of signed rotor spin rates (rad/s)  
        // For multirtor: omega_spin_sum = (+Ω1 - Ω2 + Ω3 - Ω4) depending on spin directions
    };

    // compute omega_dot from Euler equation:
    // omega_dot = I^{-1} (tau_total - omega x (I omega))
    Eigen::Vector3d omegaDotEuler(
        const Eigen::Vector3d &omega_b,
        const Eigen::Vector3d &tau_b,
        const RigidBodyParams &rb);

    // Simplified rotor gyroscopic torque:
    // tau_gyro = rotor_J*(omega_b X [0,0,omega_spin_sum]) 
    Eigen::Vector3d gyroTorqusSimplified(
        const Eigen::Vector3d &omega_b,
        const RigidBodyParams &rb);
}