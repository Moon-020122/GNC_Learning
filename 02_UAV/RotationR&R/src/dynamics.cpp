#include "dynamics.hpp"

namespace gnc
{
    Eigen::Vector3d gyroTorqusSimplified(
        const Eigen::Vector3d &omega_b,
        const RigidBodyParams &rb)
    {
        Eigen::Vector3d Omega_sum(0.0, 0.0, rb.omega_spin_sum);
        return rb.rotor_J * Omega_sum.cross(omega_b); // cross = 叉乘 cross product
    }

    Eigen::Vector3d omegaDotEuler(
        const Eigen::Vector3d &omega_b,
        const Eigen::Vector3d &tau_b,
        const RigidBodyParams &rb)
    {
        const Eigen::Vector3d Iw = rb.I_b * omega_b; //angular momentum
        //The body's own gyroscopic torque
        const Eigen::Vector3d gyro_coupling = omega_b.cross(Iw); //w x (Iw)  
        // 外力矩 + 
        const Eigen::Vector3d tau_total = tau_b + gyroTorqusSimplified(omega_b,rb);

        return rb.I_b_inv * (tau_total - gyro_coupling);
    }
};